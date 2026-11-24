namespace LegoWalkers

open System

/// Complete robot founders assembled from the catalog-realized Walker1 linkage.
module Walker1Seeds =
    let private unwrap result = Result.defaultWith (fun gate -> failwith (Realization.json gate)) result

    let private placement (p: PartInstance) : WalkerPlacement =
        { Id = p.Id; Part = p.Part; Color = p.Color; Position = p.Position; Rotation = p.Rotation }

    /// Two transverse Walker1 legs share one output axle and a remote-control chassis.
    /// This is a realizable founder recipe, not a claim of stability or walking fitness.
    /// The 8L output axle engages each crank by 10 LDU; axial retention is not certified.
    let walker feet : MotionProgram * WalkerGenome =
        if feet <> 2 then invalidArg "feet" "The Walker1 founder currently supports exactly two feet."
        let source = Realization.screen MotionExamples.walker1 72 |> unwrap
        let recipe = Realization.propose source 1 |> unwrap |> List.head
        let platform = Platform.build { Genome.defaults with Remote = true } 1 0.0
        let parts = ResizeArray<PartInstance>(platform.Parts)
        let mounts = ResizeArray<WalkerPlacement>()
        let connections = ResizeArray<WalkerPivotChoice>()
        let bodies = ResizeArray<MotionBody>()
        let hinges = ResizeArray<MotionHinge>()
        let groundSites = ResizeArray<MotionSite>()
        let outputs = ResizeArray<MotionFoot>()
        let footChoices = ResizeArray<WalkerFootChoice>()
        for index in 0..2 do
            let id, x = $"shaft-{index}", 40.0+float index*60.0
            let angle = if index%2 = 0 then 0.0 else Math.PI/24.0
            groundSites.Add (Motion.site $"bearing-{index}" x 0)
            bodies.Add (Motion.rigid id x 0 angle [Motion.site "root" 0 0])
            hinges.Add (Motion.couple $"bearing-{index}" (Motion.port "chassis" $"bearing-{index}") (Motion.port id "root"))
        for j in platform.Joints do
            let p = platform.Parts |> List.find (fun p -> p.Id = j.Connector)
            connections.Add { Group = j.Id; Hinges = [j.Id]; Placement = { placement p with Id = $"pivot-{j.Id}" } }
        let platformPivots = platform.Joints |> List.map _.Connector |> Set.ofList
        for p in platform.Parts do
            if p.Body.IsNone && not (platformPivots.Contains p.Id) then mounts.Add (placement p)
        for side in 0..1 do
            let prefix = $"leg-{side}"
            let name id = $"{prefix}-{id}"
            let fixture = Realization.compile source { recipe with Side = if side = 0 then -1 else 1 } |> unwrap
            let face = if side = 0 then 0.0 else 120.0
            let translate (p: Vec3) = Vec3.create (p.X+260.0) p.Y (p.Z+face)
            let planar (p: Vec3) = Vec3.create (p.X+260.0) p.Y 0
            let owner = function "ground" -> "chassis" | "crank" -> "shaft-2" | id -> name id
            let port (p: MotionPort) = { Body = owner p.Body; Site = if p.Body = "ground" || p.Body = "crank" then name p.Site else p.Site }
            for b in source.Program.Bodies do
                if b.Grounded then
                    for site in b.Sites do
                        if site.Id <> "drive" then groundSites.Add { site with Id = name site.Id; Position = planar site.Position }
                elif b.Id = "crank" then
                    let shaft = bodies[2]
                    bodies[2] <- { shaft with Sites = shaft.Sites @ (b.Sites |> List.map (fun s -> { s with Id = name s.Id })) }
                else
                    let pose = source.Frames.Head.Poses[b.Id]
                    bodies.Add { b with Id = owner b.Id; Initial = { pose with Origin = planar pose.Origin } }
            for h in source.Program.Hinges do
                if h.Id <> "drive" then hinges.Add { Id = name h.Id; A = port h.A; B = port h.B }
            for p in fixture.Assembly.Parts do
                match p.Body with
                | Some "ground" when p.Id.StartsWith("frame-upright-") ->
                    parts.Add { p with Id = name p.Id; Body = Some "chassis"; Position = translate p.Position }
                | Some id when id <> "ground" ->
                    parts.Add { p with Id = name p.Id; Body = Some (owner id); Position = translate p.Position }
                | None when p.Id.StartsWith("frame-upright-pin-") ->
                    mounts.Add { placement p with Id = name p.Id; Position = translate p.Position }
                | _ -> ()
            for group, hs, _ in fixture.Groups do
                if group <> "drive" then
                    let p = fixture.Assembly.Parts |> List.find (fun p -> p.Id = $"pivot-{group}")
                    connections.Add {
                        Group = name group; Hinges = hs |> List.map name
                        Placement = { placement p with Id = $"pivot-{name group}"; Position = translate p.Position }
                    }
            outputs.Add (Motion.foot prefix (name "sole") "contact" 4)
            let sole = fixture.Assembly.Parts |> List.find (fun p -> p.Id = "body-sole")
            footChoices.Add { Foot = prefix; Part = name sole.Id; Depth = (translate sole.Position).Z; AdditionalContacts = None }
        bodies.Add (Motion.ground "chassis" (List.ofSeq groundSites))
        let program = {
            Bodies = List.ofSeq bodies; Hinges = List.ofSeq hinges
            Motor = { Body = "shaft-0"; Phase = 0.0 }
            Transmissions = [for index in 1..2 -> Motion.transmit $"shaft-{index-1}" $"shaft-{index}" -1 (Math.PI/24.0)]
            Feet = List.ofSeq outputs
        }
        let genome = {
            SchemaVersion = 1; RecipeVersion = WalkerRealization.recipeVersion; MechanismId = Realization.mechanismId program
            Remote = true; TowWagon = false
            Bodies = parts |> Seq.choose (fun p -> p.Body |> Option.map (fun b -> b, p))
                     |> Seq.groupBy fst
                     |> Seq.map (fun (id, ps) -> { Body = id; Parts = ps |> Seq.map (snd >> placement) |> Seq.toList }) |> Seq.toList
            Connections = List.ofSeq connections; Mounts = List.ofSeq mounts; Feet = List.ofSeq footChoices
            MotorPart = "motor"; Speed = 3.0; MaxTorque = 0.4; LegFamilies = None
        }
        program, genome

    let sharedWalker feet =
        let program,genome = walker feet
        let instances = [
            for side in 0..1 ->
                WalkerLegFamilies.instance $"leg-{side}" $"leg-{side}" (side=1) 0.0 program
        ]
        program,WalkerLegFamilies.bind "legs" instances program genome
