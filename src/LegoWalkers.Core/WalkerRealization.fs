namespace LegoWalkers

open System
open System.IO
open System.Text.Json
open System.Text.Json.Nodes
open System.Text.Json.Serialization

type RealizedWalker = {
    Assembly: Assembly
    Genome: WalkerGenome
    GenotypeId: string
    AssemblyId: string
    Groups: (string * string list * string list) list
    LogicalFeet: Map<string, string>
}

module WalkerRealization =
    let version = "technic-motion-v3"
    let recipeVersion = "walker-parts-v1"
    let soleContactVersion = "bounded-support-surfaces-v1"
    let private options =
        JsonSerializerOptions(PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
                              UnmappedMemberHandling = JsonUnmappedMemberHandling.Disallow,
                              RespectRequiredConstructorParameters = true, WriteIndented = true)
    let json value = JsonSerializer.Serialize(value, options)
    let readGenome path =
        let document = JsonNode.Parse(File.ReadAllText path)
        match document with
        | :? JsonObject as fields ->
            if not (fields.ContainsKey "legFamilies") then fields["legFamilies"] <- null
            match fields["feet"] with
            | :? JsonArray as feet ->
                for foot in feet do
                    match foot with
                    | :? JsonObject as choice when not (choice.ContainsKey "additionalContacts") ->
                        choice["additionalContacts"] <- null
                    | _ -> ()
            | _ -> ()
        | _ -> ()
        document.Deserialize<WalkerGenome>(options)
    let private reject message = invalidArg "realization" message
    let private near a b = abs (a-b) < 1e-6
    let private canonical (g: WalkerGenome) =
        { g with
            Bodies = g.Bodies |> List.map (fun b -> { b with Parts = List.sortBy _.Id b.Parts }) |> List.sortBy _.Body
            Connections = g.Connections |> List.map (fun c -> { c with Hinges = List.sort c.Hinges }) |> List.sortBy _.Group
            Mounts = List.sortBy _.Id g.Mounts
            Feet = g.Feet |> List.map (fun foot ->
                {foot with AdditionalContacts=foot.AdditionalContacts |> Option.map (List.sortBy (fun c -> c.Part,c.Depth))})
                |> List.sortBy _.Foot
            LegFamilies = g.LegFamilies |> Option.map WalkerLegFamilies.canonical }
    let private coverage label actual expected =
        if List.length actual <> Set.count (Set.ofList actual) || Set.ofList actual <> Set.ofList expected then
            reject $"{label} must cover the motion program exactly once."
    let private part owner (p: WalkerPlacement) : PartInstance =
        { Id = p.Id; Part = p.Part; Color = p.Color; Position = p.Position; Rotation = p.Rotation; Body = owner }

    let compile (s: ScreenedMotion) (genome: WalkerGenome) =
        try
            let g = canonical genome
            WalkerLegFamilies.validateAt s.Program s.Frames.Head.Poses g
            if g.SchemaVersion <> 1 || g.RecipeVersion <> recipeVersion || g.MechanismId <> s.MechanismId then
                reject "Walker recipe, schema or mechanism identity differs from the screened motion."
            if not g.Remote || g.TowWagon then reject "Complete walkers must be remote and non-towing."
            if Motion.footCount s.Program < 2 || Motion.footCount s.Program > Motion.maximumFeet then
                reject "Complete walkers require two to six logical feet."
            if s.Program.Bodies.Length > 32 then reject "The walker compiler is bounded to 32 bodies."
            if not (Double.IsFinite g.Speed && Double.IsFinite g.MaxTorque) || g.Speed = 0.0 || g.MaxTorque <= 0.0 then
                reject "Motor speed must be finite and nonzero; torque must be finite and positive."
            let ground =
                match s.Program.Bodies |> List.filter _.Grounded with
                | [b] -> b
                | _ -> reject "A walker requires exactly one rigid chassis."
            coverage "Body choices" (g.Bodies |> List.map _.Body) (s.Program.Bodies |> List.map _.Id)
            coverage "Pivot groups" (g.Connections |> List.map _.Group) (g.Connections |> List.map _.Group)
            coverage "Hinge choices" (g.Connections |> List.collect _.Hinges) (s.Program.Hinges |> List.map _.Id)
            coverage "Foot choices" (g.Feet |> List.map _.Foot) (s.Program.Feet |> List.map _.Id)
            if g.Bodies |> List.exists (fun b -> b.Parts.IsEmpty) then reject "Every body needs concrete structural parts."
            let parts = [
                for b in g.Bodies do for p in b.Parts -> part (Some b.Body) p
                for c in g.Connections -> part None c.Placement
                for p in g.Mounts -> part None p
            ]
            if parts.Length > 512 then reject "The walker compiler is bounded to 512 parts."
            coverage "Part IDs" (parts |> List.map _.Id) (parts |> List.map _.Id)
            for p in parts do
                if String.IsNullOrWhiteSpace p.Id || not (Vec3.finite p.Position && Rotation.valid p.Rotation) then
                    reject "Every part requires a unique nonempty ID and finite proper placement."
                let connector = (Catalog.get p.Part).Connector
                if connector.IsSome <> p.Body.IsNone then reject $"Invalid connector/body assignment on {p.Id}."
            for mount in g.Mounts do
                if (Catalog.get mount.Part).Connector <> Some FrictionPin then
                    reject "Rigid attachments must use catalog friction pins; articulated connectors belong to pivot choices."
            let byId = parts |> List.map (fun p -> p.Id, p) |> Map.ofList
            for id, expected in ["payload-battery", "58119.dat"; "payload-receiver", "58123.dat"; g.MotorPart, "58121.dat"] do
                match Map.tryFind id byId with
                | Some p when p.Part = expected && p.Body = Some ground.Id -> ()
                | _ -> reject $"Required chassis hardware {id} must be an actual {expected} part."
            if parts |> List.filter (fun p -> p.Part = "58121.dat") |> List.length <> 1 then
                reject "A walker must have exactly one physical motor."
            let hinges = s.Program.Hinges |> List.map (fun h -> h.Id, h) |> Map.ofList
            let groups = ResizeArray<_>()
            let joints = [
                for c in g.Connections do
                    if c.Hinges.IsEmpty || c.Placement.Id <> $"pivot-{c.Group}" then reject "Pivot groups need hinges and matching connector IDs."
                    if (Catalog.get c.Placement.Part).Connector = Some FrictionPin then
                        reject "A moving pivot cannot use a rigid friction pin."
                    let hs = c.Hinges |> List.map (fun id -> hinges[id])
                    let members = hs |> List.collect (fun h -> [h.A.Body; h.B.Body]) |> List.distinct |> List.sort
                    if members.Length < 2 || members.Length > 3 || hs.Length <> members.Length-1 then
                        reject "A physical pivot must connect two or three bodies with N-1 hinges."
                    let axis = Rotation.apply c.Placement.Rotation (Vec3.create 1 0 0)
                    if not (near (abs axis.Z) 1.0) then reject "The planar walker recipe requires transverse hinge axes."
                    for h in hs do
                        for port in [h.A; h.B] do
                            let p = Motion.position s.Graph s.Frames.Head port |> Result.defaultWith reject
                            if not (near p.X c.Placement.Position.X && near p.Y c.Placement.Position.Y) then
                                reject $"Connector {c.Group} differs from its solved motion pivot."
                        yield { Id = h.Id; BodyA = h.A.Body; BodyB = h.B.Body
                                Position = c.Placement.Position; Axis = Vec3.create 0 0 1; Connector = c.Placement.Id }
                    groups.Add (c.Group, c.Hinges, members)
            ]
            let gears = [
                for t in s.Program.Transmissions do
                    if t.Ratio >= 0.0 then reject "Supported external spur meshes must reverse rotation."
                    let candidates id =
                        parts |> List.filter (fun p -> p.Body = Some id && (Catalog.gearTeeth p.Part).IsSome)
                        |> List.map (fun p ->
                            let pose = s.Frames.Head.Poses[id]
                            let expected = Rotation.aroundZ pose.Angle
                            if not (near p.Position.X pose.Origin.X && near p.Position.Y pose.Origin.Y)
                               || Vec3.distance p.Rotation.X expected.X > 1e-6
                               || Vec3.distance p.Rotation.Y expected.Y > 1e-6 then
                                reject "Gear placements must follow their motion body's solved axis and phase."
                            p)
                    let pairs = [
                        for a in candidates t.Input do
                            for b in candidates t.Output do
                                let na, nb = float (Catalog.gearTeeth a.Part).Value, float (Catalog.gearTeeth b.Part).Value
                                if Catalog.matchesSpurMesh t.Ratio a.Part a.Position b.Part b.Position then
                                    yield a,b,na/nb
                    ]
                    match pairs with
                    | [_,_,ratio] -> yield { BodyA = t.Input; BodyB = t.Output; Ratio = ratio }
                    | _ -> reject "Every transmission requires one unambiguous physical gear pair with matching teeth, ratio, plane and center spacing."
            ]
            let input =
                match joints |> List.filter (fun j -> Set.ofList [j.BodyA; j.BodyB] = Set.ofList [ground.Id; s.Program.Motor.Body]) with
                | [j] -> j
                | _ -> reject "The motor needs one chassis bearing on the input body."
            if (Catalog.get byId[input.Connector].Part).Connector <> Some Axle then reject "The motor input must use an axle."
            let rec mountedPlate layers (plate: PartInstance) =
                layers<=2 &&
                (parts |> List.exists (fun parent ->
                    parent.Id<>plate.Id && parent.Body=plate.Body
                    && Vec3.distance parent.Rotation.X plate.Rotation.X<1e-6
                    && Vec3.distance parent.Rotation.Y plate.Rotation.Y<1e-6
                    && Vec3.distance parent.Rotation.Z plate.Rotation.Z<1e-6
                    && (Catalog.soleUnderside parent.Part |> Option.exists (fun height ->
                        Vec3.distance (Vec3.add parent.Position (Vec3.scale height parent.Rotation.Y)) plate.Position<1e-6))
                    && (parent.Part="3701.dat" || (parent.Part="3710.dat" && mountedPlate (layers+1) parent))))
            let logicalFeet = ResizeArray<string * string>()
            let feet, paths =
                [for f in s.Program.Feet do
                    let choice = g.Feet |> List.find (fun c -> c.Foot = f.Id)
                    let primary =
                        Map.tryFind choice.Part byId
                        |> Option.defaultWith (fun () -> reject $"Missing primary foot part {choice.Part}.")
                    let contacts = WalkerFootChoice.contacts choice
                    if contacts.Length>2 || (contacts |> List.map _.Part |> Set.ofList |> Set.count)<>contacts.Length then
                        reject "A logical foot permits one primary surface and one distinct parallel inner sole."
                    for contactIndex,contact in contacts |> List.indexed do
                      let source =
                        match Map.tryFind contact.Part byId with
                        | Some p when p.Body=Some f.Port.Body && Double.IsFinite choice.Depth -> p
                        | _ -> reject "Foot choices must identify an actual part on their output body and a finite depth."
                      if not (Double.IsFinite contact.Depth) then reject "Foot contact depths must be finite."
                      if contactIndex>0 &&
                         ((Catalog.soleUnderside source.Part).IsNone || primary.Part<>source.Part
                          || not(near source.Position.X primary.Position.X && near source.Position.Y primary.Position.Y)
                          || Vec3.distance source.Rotation.X primary.Rotation.X>1e-6
                          || Vec3.distance source.Rotation.Y primary.Rotation.Y>1e-6
                          || Vec3.distance source.Rotation.Z primary.Rotation.Z>1e-6
                          || not(near source.Position.Z contact.Depth)) then
                          reject "An additional foot surface must be an aligned parallel sole on the same rigid output."
                      let port = {f.Port with Body=source.Body.Value}
                      if f.Radius > 10.0 then reject "Foot contact radii may not exceed 10 LDU."
                      let offsets =
                        if (Catalog.soleUnderside source.Part).IsSome then
                            if not (near (abs source.Rotation.Z.Z) 1.0) then reject "Sole contact patches require a transverse brick depth axis."
                            let shift =
                                if source.Part="3710.dat" then
                                    if not (mountedPlate 1 source) || not(near source.Position.Z contact.Depth) then
                                        reject "Plate soles require one or two stud-aligned 3710 plates underneath a 3701 brick at the declared depth."
                                    let original = Motion.position s.Graph s.Frames.Head port |> Result.defaultWith reject
                                    let bottom = Vec3.add source.Position (Vec3.scale 8.0 source.Rotation.Y)
                                    Vec3.subtract bottom {original with Z=contact.Depth}
                                else Vec3.zero
                            [for x in [-40.0; 40.0] do
                                for z in [-10.0; 10.0] do
                                    yield Vec3.add shift (Vec3.add (Vec3.scale x source.Rotation.X) (Vec3.scale z source.Rotation.Z))]
                        else
                            if not (near (abs source.Rotation.Y.Z) 1.0) then reject "Beam toe support requires a transverse thickness axis."
                            let low, high =
                                Realization.bounds source.Part
                                |> Option.defaultWith (fun () -> reject $"No bounded toe recipe for {source.Part}.")
                            [for depth in [low.Y; high.Y] -> Vec3.scale depth source.Rotation.Y]
                      for index, offset in offsets |> List.indexed do
                        let prefix = if contactIndex=0 then f.Id else $"{f.Id}-inner"
                        let id = if offsets.Length = 1 then prefix else $"{prefix}-{index}"
                        logicalFeet.Add (id, f.Id)
                        let points =
                            s.Frames |> List.map (fun frame ->
                                let p = Motion.position s.Graph frame port |> Result.defaultWith reject
                                let angle = frame.Poses[port.Body].Angle-s.Frames.Head.Poses[port.Body].Angle
                                Vec3.add { p with Z = contact.Depth } (Rotation.apply (Rotation.aroundZ angle) offset))
                        yield ({ Id = id; Body = port.Body; Position = points.Head; Part = contact.Part },
                               { Id = id; Points = points
                                 Stride = (points |> List.maxBy _.X).X-(points |> List.minBy _.X).X
                                 Lift = (points |> List.maxBy _.Y).Y-(points |> List.minBy _.Y).Y })]
                |> List.unzip
            let ordered =
                parts |> List.sortBy (fun p ->
                    (match (Catalog.get p.Part).Connector with
                     | None -> 0 | Some Axle -> if p.Id = input.Connector then 1 else 2
                     | Some FrictionPin -> 3 | Some FrictionlessPin -> 4), p.Id)
            let assembly = {
                Kind = Walker; Genome = { Genome.defaults with Remote = true }; Parts = ordered; Joints = joints; Gears = gears
                Drives = [{ MotorPart = g.MotorPart; Joint = input.Id; Speed = g.Speed; MaxTorque = g.MaxTorque }]
                Chassis = ground.Id; Feet = feet; FootPaths = paths
                Warnings = ["Complete motion-program walker; walking fitness has not been established."
                            "Spur gearing uses explicit 8/24/40-tooth parts; teeth contact is idealized."
                            "Electronics mass, friction-pin rigidity, axial retention and load capacity remain approximations."]
            }
            match Grammar.validateAssembly assembly with
            | errors when not errors.IsEmpty -> reject (String.concat "; " errors)
            | _ -> ()
            let identityVersion = version+soleContactVersion
            Ok { Assembly = assembly; Genome = g; AssemblyId = Export.assemblyId assembly
                 GenotypeId = Export.sha256 (identityVersion+json g); Groups = List.ofSeq groups
                 LogicalFeet = Map.ofSeq logicalFeet }
        with :? ArgumentException as error ->
            Error { Gate = "lego-realizability"; Outcome = "reject"; Code = "invalid-walker-realization"; Diagnostics = [error.Message] }

    let write directory (s: ScreenedMotion) (r: RealizedWalker) =
        let geometry = Realization.screenAssemblyGeometry s r.Assembly
        if geometry.Outcome = "reject" then reject (String.concat "; " geometry.Diagnostics)
        let directory = Path.GetFullPath directory
        if Directory.Exists directory && not (Directory.EnumerateFileSystemEntries directory |> Seq.isEmpty) then
            invalidArg "directory" "Output directory must be empty."
        Directory.CreateDirectory directory |> ignore
        let ldraw = (Export.ldraw r.Assembly).Replace($"Grammar: {Export.grammarVersion}; seed: {r.Assembly.Genome.Seed}", $"Grammar: {version}")
        let manifest = JsonNode.Parse(Export.manifest "model.ldr" r.Assembly)
        manifest["grammarVersion"] <- JsonValue.Create version
        manifest["ldrawSha256"] <- JsonValue.Create (Export.sha256 ldraw)
        manifest["genome"] <- JsonNode.Parse(json r.Genome)
        for f in manifest["feet"].AsArray() do
            let id = f["id"].GetValue<string>()
            let logical = r.LogicalFeet[id]
            f["logicalFoot"] <- JsonValue.Create logical
            f["contactRadiusMeters"] <- JsonValue.Create ((s.Program.Feet |> List.find (fun p -> p.Id = logical)).Radius*0.0004)
        manifest["motion"] <- JsonNode.Parse(Realization.json {|
            mechanismId = s.MechanismId; genotypeId = r.GenotypeId; assemblyId = r.AssemblyId
            gateVersion = Realization.gateVersion; cycleSteps = s.Frames.Length-1
            footCount = Motion.footCount s.Program
            logicalFeet = s.Program.Feet |> List.map (fun f ->
                let choice = r.Genome.Feet |> List.find (fun choice -> choice.Foot = f.Id)
                let source = r.Assembly.Parts |> List.find (fun p -> p.Id = choice.Part)
                {| id = f.Id; body = f.Port.Body
                   contactShape = if (Catalog.soleUnderside source.Part).IsSome then "rectangle" else "capsule"
                   surfaces = WalkerFootChoice.contacts choice |> List.map (fun contact ->
                       {| contactShape = if (Catalog.soleUnderside source.Part).IsSome then "rectangle" else "capsule"
                          body = r.Assembly.Parts |> List.find (fun part -> part.Id=contact.Part) |> _.Body |> Option.get
                          regions = r.Assembly.Feet |> List.choose (fun region ->
                              if region.Part=contact.Part && r.LogicalFeet[region.Id]=f.Id then Some region.Id else None) |})
                   regions = r.LogicalFeet |> Map.toList |> List.choose (fun (region,logical) -> if logical = f.Id then Some region else None) |})
            sharedPivots = r.Groups |> List.map (fun (id, hs, bs) -> {| id = id; abstractHinges = hs; bodies = bs |})
        |})
        File.WriteAllText(Path.Combine(directory, "model.ldr"), ldraw)
        File.WriteAllText(Path.Combine(directory, "manifest.json"), manifest.ToJsonString(options)+"\n")
        File.WriteAllText(Path.Combine(directory, "realization.json"), json r.Genome+"\n")
        File.WriteAllText(Path.Combine(directory, "program.json"), MotionExport.programJson s.Program)
        File.WriteAllText(Path.Combine(directory, "cycle.svg"), MotionExport.cycleSvg s.Program s.Graph s.Frames)
        let gates = s.Decisions @ [
            { Gate = "lego-realizability"; Outcome = "pass"; Code = "explicit-whole-walker-parts"; Diagnostics = [] }; geometry
        ]
        File.WriteAllText(Path.Combine(directory, "gates.json"), Realization.json gates+"\n")
        directory
