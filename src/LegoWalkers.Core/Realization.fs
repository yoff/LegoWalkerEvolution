namespace LegoWalkers

open System
open System.IO
open System.Text
open System.Text.Json
open System.Text.Json.Nodes
open System.Text.Json.Serialization

type BodyRealization = { Body: string; Part: string; Layer: int; Offset: float }
type ConnectionRealization = { Group: string; Part: string; Center: float }
type RealizationGenome = {
    SchemaVersion: int
    RecipeVersion: string
    MechanismId: string
    Side: int
    Bodies: BodyRealization list
    Connections: ConnectionRealization list
}
type GateDecision = { Gate: string; Outcome: string; Code: string; Diagnostics: string list }
type ScreenedMotion = {
    Program: MotionProgram
    Graph: CompiledMotion
    Frames: MotionFrame list
    MechanismId: string
    Decisions: GateDecision list
}
type RealizedMotion = {
    Assembly: Assembly
    Genome: RealizationGenome
    GenotypeId: string
    AssemblyId: string
    Groups: (string * string list * string list) list
    LogicalFeet: Map<string, string>
}

module Realization =
    let version = "technic-motion-v2"
    let recipeVersion = "planar-parts-v1"
    let gateVersion = "motion-gates-v1"
    let private options =
        JsonSerializerOptions(PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
                              UnmappedMemberHandling = JsonUnmappedMemberHandling.Disallow, WriteIndented = true)
    let json value = JsonSerializer.Serialize(value, options)
    let private decision gate outcome code diagnostics =
        { Gate = gate; Outcome = outcome; Code = code; Diagnostics = diagnostics }
    let private reject message = invalidArg "realization" message
    let private near a b = abs (a-b) < 1e-6
    let private point x y = Vec3.create x y 0
    let private canonical program =
        { program with
            Bodies = program.Bodies |> List.sortBy _.Id |> List.map (fun b -> { b with Sites = List.sortBy _.Id b.Sites })
            Hinges = program.Hinges |> List.map (fun h -> if h.A < h.B then h else { h with A = h.B; B = h.A }) |> List.sortBy _.Id
            Feet = program.Feet |> List.sortBy _.Id
            Transmissions = program.Transmissions |> List.sortBy (fun t -> t.Input, t.Output) }
    let mechanismId program = Export.sha256 ("planar-motion-program-v1" + MotionExport.programJson (canonical program))

    let screen program steps =
        if steps < 36 || steps > 3600 then invalidArg "steps" "Cycle steps must be between 36 and 3600."
        let program = canonical program
        match Motion.compile program with
        | Error errors -> Error (decision "structure" "reject" "invalid-constraint-graph" errors)
        | Ok graph ->
            match Motion.sample graph [for n in 0..steps -> float n*2.0*Math.PI/float steps] with
            | Error error -> Error (decision "mobility" "reject" "cycle-does-not-close" [error])
            | Ok frames ->
                let first, last = frames.Head, List.last frames
                let closed =
                    program.Bodies |> List.forall (fun body ->
                        let a, b = first.Poses[body.Id], last.Poses[body.Id]
                        Vec3.distance a.Origin b.Origin < 1e-5
                        && abs (atan2 (sin (a.Angle-b.Angle)) (cos (a.Angle-b.Angle))) < 1e-6)
                let diagnostics = [
                        for foot in program.Feet do
                            let positions = frames |> List.map (fun f -> (Motion.footPositions graph f)[foot.Id])
                            let stride = (positions |> List.maxBy _.X).X - (positions |> List.minBy _.X).X
                            let lift = (positions |> List.maxBy _.Y).Y - (positions |> List.minBy _.Y).Y
                            yield $"{foot.Id}: stride={stride:G6} LDU; lift={lift:G6} LDU (heuristic, not walking fitness)."
                ]
                Ok { Program = program; Graph = graph; Frames = frames; MechanismId = mechanismId program
                     Decisions = [
                         decision "structure" "pass" "valid-constraint-graph" []
                         decision "mobility" (if closed then "pass" else "warning") "sampled-input-revolution"
                             [$"{steps+1} frames; all hinge constraints close; endpoint repeats={closed}; sampling is not a continuous proof."
                              "A nonrepeating endpoint may indicate a longer-period mechanism, not an invalid design."]
                         decision "motion-potential" "warning" "unranked-kinematics" diagnostics
                     ] }

    let private body (s: ScreenedMotion) id = s.Program.Bodies |> List.find (fun b -> b.Id = id)
    let private site s (port: MotionPort) = (body s port.Body).Sites |> List.find (fun p -> p.Id = port.Site) |> _.Position
    let private position (s: ScreenedMotion) (port: MotionPort) =
        Motion.position s.Graph s.Frames.Head port |> Result.defaultWith reject
    let private requiredPorts s id =
        [for h in s.Program.Hinges do yield h.A; yield h.B]
        |> List.filter (fun p -> p.Body = id) |> List.distinct
    let private grounded (s: ScreenedMotion) =
        match s.Program.Bodies |> List.filter _.Grounded with
        | [ground] -> ground
        | _ -> reject "This catalog requires exactly one grounded support body."
    let private driveHinge s =
        let ground = grounded s
        match s.Program.Hinges |> List.filter (fun h ->
            Set.ofList [h.A.Body; h.B.Body] = Set.ofList [ground.Id; s.Program.Motor.Body]) with
        | [hinge] -> hinge
        | _ -> reject "The direct-drive recipe needs one hinge between the input body and the support frame."
    let private drivePorts s =
        let h = driveHinge s
        if h.A.Body = s.Program.Motor.Body then h.B, h.A else h.A, h.B
    let private drivePosition s = drivePorts s |> fst |> position s

    // Coincident sites on the same rigid body identify one physical multi-body pivot.
    let private groups s =
        let samePort (a: MotionPort) (b: MotionPort) = a.Body = b.Body && Vec3.distance (site s a) (site s b) < 1e-7
        let touches (a: MotionHinge list) (b: MotionHinge list) =
            a |> List.exists (fun x -> b |> List.exists (fun y ->
                [x.A; x.B] |> List.exists (fun p -> [y.A; y.B] |> List.exists (samePort p))))
        let rec collect acc = function
            | [] -> acc
            | first :: rest ->
                let rec grow current remaining =
                    let joined, separate = remaining |> List.partition (touches current)
                    if joined.IsEmpty then current, separate
                    else grow (current @ List.concat joined) separate
                let merged, remaining = grow first rest
                collect (merged :: acc) remaining
        collect [] (s.Program.Hinges |> List.map List.singleton)
        |> List.map (List.sortBy _.Id) |> List.sortBy (fun hs -> hs.Head.Id)
    let private groupPorts hs =
        hs |> List.collect (fun (h: MotionHinge) -> [h.A; h.B]) |> List.distinctBy _.Body
    let private groundDepth s (port: MotionPort) =
        if port.Body <> (grounded s).Id then reject "Expected a support port."
        let p, root = position s port, drivePosition s
        if near p.Y root.Y then 0 else 1
    let private candidatesForBody s id =
        let b = body s id
        let ports = requiredPorts s id
        let points = ports |> List.map (site s)
        let feet = s.Program.Feet |> List.filter (fun f -> f.Port.Body = id)
        let footPoints = feet |> List.map (fun f -> site s f.Port)
        if b.Grounded then ["3703.dat", 0.0]
        elif points |> List.exists (fun p -> not (near p.Y 0.0)) then
            if footPoints.IsEmpty && (points |> List.distinct |> List.length) >= 2
               && not (Catalog.fitPlanarPart "32526.dat" (Vec3.create 0 0 1) points).IsEmpty then
                ["32526.dat", 0.0]
            else []
        elif footPoints |> List.exists (fun p -> not (near p.Y 0.0)) then
            let center = footPoints.Head.X
            if id <> s.Program.Motor.Body
               && points |> List.forall (fun p -> [-20.0; 0.0; 20.0] |> List.exists (near (p.X-center)))
               && footPoints |> List.forall (fun p -> near p.X center && near p.Y 14.0)
            then ["3701.dat", center] else []
        else
            [for part in Catalog.planarBeamParts do
                let spec = Catalog.get part
                for first in spec.Ports do
                    let offset = points.Head.X-first.Position.Z
                    let matches (p: Vec3) =
                        spec.Ports |> List.exists (fun hole -> near (hole.Position.Z+offset) p.X)
                    let isDrive =
                        if id <> s.Program.Motor.Body then true
                        else
                            let root = drivePorts s |> snd |> site s
                            spec.Ports |> List.exists (fun hole -> hole.Kind = AxleHole && near (hole.Position.Z+offset) root.X)
                    let lo = spec.Ports |> List.minBy _.Position.Z |> _.Position.Z
                    let hi = spec.Ports |> List.maxBy _.Position.Z |> _.Position.Z
                    if isDrive && List.forall matches points
                       && footPoints |> List.forall (fun p -> p.X >= lo+offset-1e-7 && p.X <= hi+offset+1e-7)
                    then yield part, offset]
            |> List.distinct

    let private validateFrame s =
        let root = drivePosition s
        let ground = grounded s
        if s.Program.Transmissions.Length <> 0 then reject "Transmission realization is not yet in the planar parts catalog."
        if s.Program.Bodies.Length > 32 then reject "The current physical compiler is bounded to 32 bodies."
        let movingRoot = drivePorts s |> snd |> site s
        if Vec3.distance movingRoot Vec3.zero > 1e-7 then reject "The direct-drive body's axle socket must be its local origin."
        let raised = ResizeArray<Vec3>()
        for port in requiredPorts s ground.Id do
            let p = position s port
            let x = (p.X-root.X)/20.0
            if not (near x (Math.Round x)) || p.X < root.X-100.0 || p.X > root.X+180.0 then
                reject "Support anchors must fit the frame's 20 LDU hole grid."
            if not (near p.Y root.Y) then
                let height = (root.Y-p.Y)/20.0
                if height < 1.0-1e-6 || height > 10.0+1e-6 || not (near height (Math.Round height)) then
                    reject "Raised supports must lie above the rail on its 20 LDU grid, within a 200 LDU upright."
                if near p.X root.X then reject "A raised support cannot overlap the drive shaft."
                raised.Add p
        if raised |> Seq.distinctBy _.X |> Seq.length > 1 then reject "This frame recipe supports at most one raised anchor."

    let private upright s =
        let root = drivePosition s
        requiredPorts s (grounded s).Id |> List.map (position s)
        |> List.filter (fun p -> not (near p.Y root.Y))
        |> List.groupBy _.X
        |> List.map (fun (_,points) ->
            let top = points |> List.minBy _.Y
            let part,length =
                ["32316.dat",80.0; "32524.dat",120.0; "32525.dat",200.0]
                |> List.find (fun (_,length) -> length >= root.Y-top.Y-1e-6)
            point top.X (root.Y-length/2.0),part)

    let private completeLayers s preferred =
        let ground = (grounded s).Id
        let input = s.Program.Motor.Body
        let pivots = groups s |> List.map groupPorts
        let feasible assigned =
            pivots |> List.forall (fun ports ->
                let depths = ports |> List.choose (fun port ->
                    if port.Body = ground then Some (groundDepth s port) else Map.tryFind port.Body assigned)
                depths.Length = (depths |> List.distinct |> List.length)
                && (depths.IsEmpty || List.max depths-List.min depths <= 2))
        if feasible preferred then preferred
        else
            let mutable visited = 0
            let rec search (assigned: Map<string,int>) =
                if visited >= 4096 then None
                elif assigned.Count = s.Program.Bodies.Length then Some assigned
                else
                    visited <- visited+1
                    let domains =
                        s.Program.Bodies |> List.filter (fun b -> not (assigned.ContainsKey b.Id))
                        |> List.map (fun b ->
                            b.Id, [2..16] |> List.filter (fun depth -> feasible (Map.add b.Id depth assigned))
                                  |> List.sortBy (fun depth -> abs (depth-preferred[b.Id]),depth))
                        |> List.sortBy (fun (id,values) -> values.Length,id)
                    let id,values = domains.Head
                    values |> List.tryPick (fun depth -> search (Map.add id depth assigned))
            search (Map.ofList [ground,0;input,2])
            |> Option.defaultWith (fun () ->
                reject "No supported nonoverlapping depth assignment found within 4096 partial assignments.")

    let private inputAxle = function
        | 1 -> "3705.dat",-10.0
        // The mesh-derived 7L tip ends flush with the layer-2 input link's outer face.
        | 2 -> "44294.dat",-19.5
        | _ -> reject "This motor/frame recipe supports input layer 1 or 2."

    let private proposeBase s =
        validateFrame s
        let ground = grounded s
        let driveId = (driveHinge s).Id
        let mutable layers = Map.ofList [ground.Id, 0; s.Program.Motor.Body, 1]
        let mutable remaining = groups s |> List.filter (fun hs -> hs |> List.forall (fun h -> h.Id <> driveId))
        while not remaining.IsEmpty do
            let ready, later = remaining |> List.partition (fun hs -> groupPorts hs |> List.exists (fun p -> layers.ContainsKey p.Body))
            if ready.IsEmpty then reject "Could not assign connected depth layers."
            for hs in ready do
                let ports =
                    groupPorts hs |> List.sortBy (fun p ->
                        -(hs |> List.sumBy (fun h -> (if h.A.Body = p.Body then 1 else 0)+(if h.B.Body = p.Body then 1 else 0))),
                        p.Body)
                let known =
                    ports |> List.choose (fun p ->
                        if p.Body = ground.Id then Some (groundDepth s p)
                        else Map.tryFind p.Body layers)
                let mutable next = List.max known + 1
                for port in ports do
                    if not (layers.ContainsKey port.Body) then
                        layers <- Map.add port.Body next layers
                        next <- next+1
            remaining <- later
        let layers = completeLayers s layers
        let choices =
            [for b in s.Program.Bodies do
                match candidatesForBody s b.Id with
                | [] -> reject $"No part/hole/foot recipe implements body {b.Id}."
                | (part, offset) :: _ -> yield { Body = b.Id; Part = part; Layer = layers[b.Id]; Offset = offset }]
        let connections =
            [for hs in groups s do
                let ports = groupPorts hs
                let depths = ports |> List.map (fun p -> if p.Body = ground.Id then groundDepth s p else layers[p.Body])
                let lo, hi = List.min depths, List.max depths
                let drive = hs |> List.exists (fun h -> h.Id = driveId)
                if drive && hs.Length <> 1 then reject "The drive pivot currently supports only the frame and input link."
                let part,center =
                    if drive then inputAxle layers[s.Program.Motor.Body]
                    else (if hi-lo <= 1 && ports.Length = 2 then "3673.dat" else "32556.dat"),float (lo+hi)*10.0
                yield { Group = hs.Head.Id; Part = part; Center = center }]
        { SchemaVersion = 1; RecipeVersion = recipeVersion; MechanismId = s.MechanismId
          Side = 1; Bodies = choices; Connections = connections }

    let propose s limit =
        if limit < 1 || limit > 32 then invalidArg "limit" "Realization limit must be between 1 and 32."
        try
            let first = proposeBase s
            let variants = seq {
                yield first
                yield { first with Side = -1 }
                for choice in first.Bodies do
                    for part, offset in candidatesForBody s choice.Body |> List.skip 1 do
                        let bodies =
                            first.Bodies |> List.map (fun b ->
                                if b.Body = choice.Body then { b with Part = part; Offset = offset } else b)
                        yield { first with Bodies = bodies }
            }
            Ok (variants |> Seq.truncate limit |> Seq.toList)
        with :? ArgumentException as error ->
            Error (decision "lego-realizability" "reject" "unsupported-recipe" [error.Message])

    let readGenome path =
        use doc = JsonDocument.Parse(File.ReadAllText path)
        let fields expected (value: JsonElement) =
            if value.ValueKind <> JsonValueKind.Object then reject "Genome records must be objects."
            let names = value.EnumerateObject() |> Seq.map _.Name |> Seq.toList
            if names.Length <> List.length expected || Set.ofList names <> Set.ofList expected then
                let required = String.concat ", " expected
                reject $"Incomplete or unknown realization fields: {required}."
        let array (value: JsonElement) =
            if value.ValueKind <> JsonValueKind.Array then reject "Body and connection choices must be arrays."
            value.EnumerateArray()
        fields ["schemaVersion"; "recipeVersion"; "mechanismId"; "side"; "bodies"; "connections"] doc.RootElement
        for value in array (doc.RootElement.GetProperty("bodies")) do fields ["body"; "part"; "layer"; "offset"] value
        for value in array (doc.RootElement.GetProperty("connections")) do fields ["group"; "part"; "center"] value
        doc.RootElement.Deserialize<RealizationGenome>(options)

    let compile (s: ScreenedMotion) (g: RealizationGenome) =
        try
            validateFrame s
            if g.SchemaVersion <> 1 || g.RecipeVersion <> recipeVersion || g.MechanismId <> s.MechanismId then
                reject "Genome schema, recipe version, or mechanism identity differs from the screened motion."
            if g.Side <> 1 && g.Side <> -1 then reject "Realization side must be +1 or -1."
            let ids = g.Bodies |> List.map _.Body
            if ids.Length <> Set.count (Set.ofList ids) || Set.ofList ids <> Set.ofList (s.Program.Bodies |> List.map _.Id) then
                reject "Body realization choices must cover every body exactly once."
            let hsGroups = groups s
            let groupIds = g.Connections |> List.map _.Group
            if groupIds.Length <> Set.count (Set.ofList groupIds)
               || Set.ofList groupIds <> Set.ofList (hsGroups |> List.map (fun hs -> hs.Head.Id)) then
                reject "Connector choices must cover every shared pivot exactly once."
            for choice in g.Bodies do
                if choice.Layer < 0 || choice.Layer > 16 || not (Double.IsFinite choice.Offset) then reject "Invalid depth layer or part offset."
                if not (candidatesForBody s choice.Body |> List.exists (fun (part, offset) -> part = choice.Part && near offset choice.Offset)) then
                    reject $"Part/hole/foot choice does not implement {choice.Body}."
            let choices = g.Bodies |> List.map (fun b -> b.Body, b) |> Map.ofList
            let ground = grounded s
            if choices[ground.Id].Layer <> 0 then reject "This motor/frame recipe requires frame layer 0."
            let axlePart,_ = inputAxle choices[s.Program.Motor.Body].Layer
            let normal = Vec3.create 0 0 (float g.Side)
            let world (port: MotionPort) =
                let p = position s port
                let layer = if port.Body = ground.Id then groundDepth s port else choices[port.Body].Layer
                { p with Z = float (g.Side*layer)*20.0 }
            let root = drivePosition s
            let parts, joints, feet = ResizeArray<PartInstance>(), ResizeArray<Joint>(), ResizeArray<Foot>()
            let add id part color pos rot owner =
                parts.Add { Id = id; Part = part; Color = color; Position = pos; Rotation = rot; Body = owner }
            let shaft = if g.Side = 1 then Rotation.shaft 0.0 else { Rotation.shaft 0.0 with X = Vec3.create 0 0 -1; Z = Vec3.create 0 -1 0 }
            let at (p: Vec3) depth = { p with Z = float g.Side*depth }
            let frameRotation = if g.Side = 1 then Rotation.identity else { Rotation.identity with X = Vec3.create -1 0 0; Z = Vec3.create 0 0 -1 }
            add "frame-rail" "3703.dat" 0 (at (Vec3.add root (point 40 -10)) 0.0) frameRotation (Some ground.Id)
            for index, (center,part) in upright s |> List.indexed do
                let bottom = point center.X root.Y
                let direction = point 0 -1
                add $"frame-upright-{index}" part 0 (at center 20.0)
                    (Rotation.beamWithNormal normal direction) (Some ground.Id)
                add $"frame-upright-pin-{index}" "4459.dat" 1 (at bottom 10.0) shaft None
            let motorRotation = { Rotation.identity with X = Vec3.create (float -g.Side) 0 0; Z = Vec3.create 0 0 (float -g.Side) }
            add "input-motor" "58121.dat" 7 (at root -30.0) motorRotation (Some ground.Id)
            add "motor-mount" "32523.dat" 0 (at root -20.0) (Rotation.beamWithNormal normal (point 1 0)) (Some ground.Id)
            for index, x in [-20.0; 20.0] |> List.indexed do
                add $"motor-mount-pin-{index}" "6558.dat" 1 (at (Vec3.add root (point x 0)) -20.0) shaft None
            for b in s.Program.Bodies |> List.filter (fun b -> not b.Grounded) do
                let choice = choices[b.Id]
                let pose = s.Frames.Head.Poses[b.Id]
                let direction = point (cos pose.Angle) (sin pose.Angle)
                let pos,rotation =
                    if choice.Part = "32526.dat" then
                        let points = requiredPorts s b.Id |> List.map (position s >> fun p -> at p (float choice.Layer*20.0))
                        match Catalog.fitPlanarPart choice.Part normal points with
                        | placement :: _ -> placement
                        | [] -> reject $"No proper molded-L placement implements {b.Id}."
                    else
                        let rot, local =
                            if choice.Part = "3701.dat" then Rotation.aroundZ pose.Angle, point choice.Offset -10
                            else Rotation.beamWithNormal normal direction, point choice.Offset 0
                        let rotation =
                            if choice.Part = "3701.dat" && g.Side = -1 then
                                { rot with X = Vec3.scale -1.0 rot.X; Z = Vec3.create 0 0 -1 }
                            else rot
                        // A mirrored brick reverses local X, so its center offset remains in mechanism coordinates.
                        (Motion.transform pose local |> fun p -> at p (float choice.Layer*20.0)),rotation
                add $"body-{b.Id}" choice.Part (if b.Id = s.Program.Motor.Body then 4 else 14) pos rotation (Some b.Id)
            let socketPart (port: MotionPort) =
                if port.Body <> ground.Id then $"body-{port.Body}"
                elif groundDepth s port = 0 then "frame-rail"
                else "frame-upright-0"
            for hs in hsGroups do
                let choice = g.Connections |> List.find (fun c -> c.Group = hs.Head.Id)
                if not (Double.IsFinite choice.Center) then reject "Non-finite connector center."
                let ports = groupPorts hs
                let drive = hs |> List.exists (fun h -> h.Id = (driveHinge s).Id)
                if drive && choice.Part <> axlePart then reject "The motor input requires the axle recipe for its depth layer."
                if not drive && not (List.contains choice.Part ["3673.dat"; "32556.dat"]) then reject "Passive pivots require a supported smooth pin."
                let p = world ports.Head
                if ports |> List.exists (fun port -> let q = world port in not (near q.X p.X && near q.Y p.Y)) then
                    reject "Declared shared pivot sockets are not coaxial."
                let depths = ports |> List.map (fun port -> (world port).Z*float g.Side)
                if List.distinct depths |> List.length <> ports.Length then reject $"Overlapping socket layers at {choice.Group}."
                let lo, hi = Catalog.shaftRange choice.Part
                let socketWidth (port: MotionPort) =
                    match choices[port.Body].Part with "32063.dat" | "6632.dat" -> 10.0 | _ -> 20.0
                for port in ports do
                    let depth = (world port).Z*float g.Side
                    let half = socketWidth port/2.0
                    if depth-half < choice.Center+lo-1e-7 || depth+half > choice.Center+hi+1e-7 then
                        reject $"Connector {choice.Group} does not span its complete socket stack."
                if ports.Length > (if choice.Part = "3673.dat" then 2 else 3) then reject "Too many bodies on one connector."
                let connectorId = $"pivot-{choice.Group}"
                add connectorId choice.Part 7 (at p choice.Center) shaft None
                let ordered =
                    ports |> List.sortBy (fun port -> parts |> Seq.findIndex (fun p -> p.Id = socketPart port))
                for index, (a, b) in List.pairwise ordered |> List.indexed do
                    joints.Add { Id = $"joint-{choice.Group}-{index}"; BodyA = a.Body; BodyB = b.Body
                                 Position = at p choice.Center; Axis = normal; Connector = connectorId }
            let paths = ResizeArray<FootPath>()
            let logicalFeet = ResizeArray<string * string>()
            for f in s.Program.Feet do
                let choice = choices[f.Port.Body]
                let centers =
                    if choice.Part = "3701.dat" then
                        [for x in [choice.Offset-40.0; choice.Offset+40.0] do
                            for depth in [-10.0; 10.0] -> point x 14, depth]
                    else [site s f.Port, 0.0]
                for index, (local, depth) in centers |> List.indexed do
                    let id = if centers.Length = 1 then f.Id else $"{f.Id}-{index}"
                    let positions =
                        s.Frames |> List.map (fun frame ->
                            let p = Motion.transform frame.Poses[f.Port.Body] local
                            { p with Z = float (g.Side*choice.Layer)*20.0+depth })
                    feet.Add { Id = id; Body = f.Port.Body; Position = positions.Head; Part = $"body-{f.Port.Body}" }
                    logicalFeet.Add (id, f.Id)
                    paths.Add { Id = id; Points = positions
                                Stride = (positions |> List.maxBy _.X).X - (positions |> List.minBy _.X).X
                                Lift = (positions |> List.maxBy _.Y).Y - (positions |> List.minBy _.Y).Y }
            let input = joints |> Seq.find (fun j -> j.Connector = $"pivot-{(driveHinge s).Id}")
            let assembly = {
                Kind = Leg; Genome = Genome.defaults; Parts = List.ofSeq parts; Joints = List.ofSeq joints; Gears = []
                Drives = [{ MotorPart = "input-motor"; Joint = input.Id; Speed = 3.0; MaxTorque = 0.4 }]
                Feet = List.ofSeq feet; FootPaths = List.ofSeq paths; Chassis = ground.Id
                Warnings = ["Single leg mechanism, not a qualified walking robot."
                            "Input axle retention uses the crank's cross-hole fit, without an added axle stop."
                            "Friction pins model rigid attachment; LEGO tolerances, axial retention and load capacity are not certified."
                            "Sampled planar closure does not establish clearance or dynamic walking."] }
            if parts |> Seq.exists (fun p -> not (Rotation.valid p.Rotation)) then reject "Invalid recipe rotation."
            let physicalLines =
                Export.ldraw assembly |> _.Split('\n') |> Array.filter (fun line -> line.StartsWith "1 ")
                |> Array.map _.Trim() |> Array.sort |> String.concat "\n"
            let genome = { g with Bodies = List.sortBy _.Body g.Bodies; Connections = List.sortBy _.Group g.Connections }
            Ok { Assembly = assembly; Genome = genome; GenotypeId = Export.sha256 (json genome)
                 AssemblyId = Export.sha256 physicalLines
                 Groups = hsGroups |> List.map (fun hs -> hs.Head.Id, hs |> List.map _.Id, groupPorts hs |> List.map _.Body)
                 LogicalFeet = Map.ofSeq logicalFeet }
        with :? ArgumentException as error ->
            Error (decision "lego-realizability" "reject" "invalid-realization" [error.Message])

    let internal bounds part =
        let pair a b = Some (a, b)
        match part with
        | "6536.dat" -> pair (Vec3.create -10 -9 -10) (Vec3.create 10 29 10)
        | "60483.dat" -> pair (Vec3.create -9 -10 -9) (Vec3.create 9 10 29)
        | "32523.dat" -> pair (Vec3.create -9 -10 -29) (Vec3.create 9 10 29)
        | "32316.dat" -> pair (Vec3.create -9 -10 -49) (Vec3.create 9 10 49)
        | "32524.dat" -> pair (Vec3.create -9 -10 -69) (Vec3.create 9 10 69)
        | "32525.dat" -> pair (Vec3.create -9 -10 -109) (Vec3.create 9 10 109)
        | "32526.dat" -> pair (Vec3.create -9 -10 -9) (Vec3.create 49 10 89)
        | "32271.dat" -> pair (Vec3.create -9 -10 -9) (Vec3.create 41 10 153)
        | "32063.dat" -> pair (Vec3.create -9 -5 -59) (Vec3.create 9 5 59)
        | "3701.dat" -> pair (Vec3.create -40 -4 -10) (Vec3.create 40 24 10)
        | "3710.dat" -> pair (Vec3.create -40 -4 -10) (Vec3.create 40 8 10)
        | "3703.dat" -> pair (Vec3.create -160 -4 -10) (Vec3.create 160 24 10)
        | _ -> None

    let private corners (p: PartInstance) =
        match bounds p.Part with
        | None -> []
        | Some (lo, hi) ->
            [for x in [lo.X; hi.X] do
                for y in [lo.Y; hi.Y] do
                    for z in [lo.Z; hi.Z] -> Vec3.add p.Position (Rotation.apply p.Rotation (Vec3.create x y z))]

    let screenAssemblyGeometry (s: ScreenedMotion) (assembly: Assembly) =
        let structural = assembly.Parts |> List.filter (fun p -> p.Body.IsSome)
        let duplicate =
            structural |> List.tryFind (fun a -> structural |> List.exists (fun b ->
                a.Id <> b.Id && a.Body <> b.Body && a.Part = b.Part
                && Vec3.distance a.Position b.Position < 1e-8 && a.Rotation = b.Rotation))
        match duplicate with
        | Some part -> decision "geometry" "reject" "coincident-structural-parts" [part.Id]
        | None ->
            let overlaps = Collections.Generic.HashSet<string>()
            let envelopes =
                structural |> List.choose (fun p ->
                    let points = corners p
                    if points.IsEmpty then None else Some (p, points))
            for frame in s.Frames do
                let boxes = [
                    for p, points in envelopes do
                        let id = p.Body.Value
                        let initial, current = s.Frames.Head.Poses[id], frame.Poses[id]
                        let transformed =
                            points |> List.map (fun p ->
                                let local = Rotation.apply (Rotation.aroundZ -initial.Angle) (Vec3.subtract p initial.Origin)
                                Motion.transform current local)
                        yield p, ((transformed |> List.minBy _.X).X, (transformed |> List.maxBy _.X).X,
                                  (transformed |> List.minBy _.Y).Y, (transformed |> List.maxBy _.Y).Y,
                                  (points |> List.minBy _.Z).Z, (points |> List.maxBy _.Z).Z)
                ]
                for i in 0..boxes.Length-1 do
                    let a, (ax, aX, ay, aY, az, aZ) = boxes[i]
                    for j in i+1..boxes.Length-1 do
                        let b, (bx, bX, by, bY, bz, bZ) = boxes[j]
                        if a.Body <> b.Body && min aX bX-max ax bx > 1e-6
                           && min aY bY-max ay by > 1e-6 && min aZ bZ-max az bz > 1e-6 then
                            overlaps.Add (String.concat " / " (List.sort [a.Id; b.Id])) |> ignore
            decision "geometry" (if overlaps.Count = 0 then "pass" else "warning") "sampled-envelopes-not-proof"
                [yield $"{s.Frames.Length} sampled poses; {overlaps.Count} potentially overlapping part-envelope pairs."
                 yield "Conservative broad phase only; motor housing, connectors and exact surface clearance require simulator checks."
                 yield! overlaps |> Seq.sort |> Seq.truncate 32]

    let screenGeometry s (r: RealizedMotion) = screenAssemblyGeometry s r.Assembly

    let write directory (s: ScreenedMotion) (r: RealizedMotion) =
        let geometry = screenGeometry s r
        if geometry.Outcome = "reject" then reject (String.concat "; " geometry.Diagnostics)
        let directory = Path.GetFullPath directory
        if Directory.Exists directory && not (Directory.EnumerateFileSystemEntries directory |> Seq.isEmpty) then
            invalidArg "directory" $"Output directory must be empty: {directory}"
        Directory.CreateDirectory directory |> ignore
        let encoding = UTF8Encoding false
        let ldraw = Export.ldraw r.Assembly |> fun text -> text.Replace($"Grammar: {Export.grammarVersion}; seed: {r.Assembly.Genome.Seed}", $"Grammar: {version}")
        let manifest = JsonNode.Parse(Export.manifest "model.ldr" r.Assembly)
        manifest["grammarVersion"] <- JsonValue.Create version
        manifest["ldrawSha256"] <- JsonValue.Create (Export.sha256 ldraw)
        manifest["genome"] <- JsonNode.Parse(json r.Genome)
        manifest["payloads"] <- JsonArray()
        manifest["groundWheels"] <- JsonArray()
        for foot in manifest["feet"].AsArray() do
            foot["logicalFoot"] <- JsonValue.Create r.LogicalFeet[foot["id"].GetValue<string>()]
        manifest["motion"] <- JsonNode.Parse(json {|
            mechanismId = s.MechanismId; genotypeId = r.GenotypeId; assemblyId = r.AssemblyId
            gateVersion = gateVersion; cycleSteps = s.Frames.Length-1
            footCount = Motion.footCount s.Program
            logicalFeet = [
                for foot in s.Program.Feet ->
                    {| id = foot.Id; body = foot.Port.Body
                       regions = r.LogicalFeet |> Map.toList |> List.choose (fun (region, id) -> if id = foot.Id then Some region else None) |}
            ]
            sharedPivots = r.Groups |> List.map (fun (id, hinges, bodies) -> {| id = id; abstractHinges = hinges; bodies = bodies |})
        |})
        File.WriteAllText(Path.Combine(directory, "model.ldr"), ldraw, encoding)
        File.WriteAllText(Path.Combine(directory, "manifest.json"), manifest.ToJsonString(options)+"\n", encoding)
        File.WriteAllText(Path.Combine(directory, "realization.json"), json r.Genome+"\n", encoding)
        File.WriteAllText(Path.Combine(directory, "program.json"), MotionExport.programJson s.Program, encoding)
        File.WriteAllText(Path.Combine(directory, "cycle.svg"), MotionExport.cycleSvg s.Program s.Graph s.Frames, encoding)
        let gates =
            s.Decisions @ [decision "lego-realizability" "pass" "explicit-parts-holes-layers-connectors" []; geometry]
        File.WriteAllText(Path.Combine(directory, "gates.json"), json gates+"\n", encoding)
        directory
