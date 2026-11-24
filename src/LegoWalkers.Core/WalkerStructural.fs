namespace LegoWalkers

open System

type WalkerPhysicalEdit = {
    Program: MotionProgram
    Genome: WalkerGenome
    Operation: string
    Changes: (string * float list * float list) list
}

module WalkerStructural =
    let private near a b = abs (a-b) < 1e-6
    let private shift delta (p: WalkerPlacement) = { p with Position = Vec3.add p.Position delta }
    let private sockets (p: WalkerPlacement) =
        (Catalog.get p.Part).Ports |> List.map (fun socket ->
            socket, Vec3.add p.Position (Rotation.apply p.Rotation socket.Position))
    let internal engagedPorts (pin: WalkerPlacement) (p: WalkerPlacement) =
        let lo,hi = Catalog.shaftRange pin.Part
        sockets p |> List.filter (fun (socket, point) ->
            let delta = Vec3.subtract point pin.Position
            let along = Vec3.dot delta pin.Rotation.X
            abs (Vec3.dot (Rotation.apply p.Rotation socket.Axis) pin.Rotation.X) > 1.0-1e-6
            && Vec3.length (Vec3.cross delta pin.Rotation.X) < 1e-6
            && along >= lo-1e-6 && along <= hi+1e-6)
    let internal engaged (pin: WalkerPlacement) (p: WalkerPlacement) =
        not (engagedPorts pin p).IsEmpty
    let private point (s: ScreenedMotion) port =
        Motion.position s.Graph s.Frames.Head port |> Result.defaultWith invalidOp
    let internal protectedBodies (p: MotionProgram) =
        p.Bodies |> List.filter _.Grounded |> List.map _.Id
        |> Set.ofList |> Set.add p.Motor.Body
        |> Set.union (p.Transmissions |> List.collect (fun t -> [t.Input;t.Output]) |> Set.ofList)
    let internal branchMembers (p: MotionProgram) start =
        let protectedIds = protectedBodies p
        let rec grow visited todo =
            match todo with
            | [] -> visited
            | id::rest when Set.contains id visited || Set.contains id protectedIds -> grow visited rest
            | id::rest ->
                let neighbors = p.Hinges |> List.choose (fun h ->
                    if h.A.Body = id then Some h.B.Body elif h.B.Body = id then Some h.A.Body else None)
                grow (Set.add id visited) (neighbors@rest)
        grow Set.empty [start]
    let private renamed prefix id = prefix+"--"+id
    let private prefix (p: MotionProgram) =
        Seq.initInfinite (fun n -> $"branch-{n}")
        |> Seq.find (fun name -> p.Bodies |> List.forall (fun b -> not (b.Id.StartsWith(name+"--"))))
    let private supportedBeam part =
        List.contains part ["32523.dat";"32316.dat";"32524.dat";"32525.dat";"32063.dat";"32271.dat"]

    let footEdits (s: ScreenedMotion) (g: WalkerGenome) = [
        let chassis = s.Program.Bodies |> List.find _.Grounded
        let chassisParts = g.Bodies |> List.find (fun b -> b.Body = chassis.Id) |> _.Parts
        let center = (chassisParts |> List.averageBy _.Position.Z)
        let chassisLevel =
            chassis.Sites |> List.map (fun site -> (Motion.transform s.Frames.Head.Poses[chassis.Id] site.Position).Y) |> List.max
        let templates = [
            "32524.dat", Vec3.create 0 0 60, "lengthen-foot"
            "32525.dat", Vec3.create 0 0 100, "lengthen-foot"
            "32271.dat", Vec3.create 32 0 144, "bend-foot"
        ]
        for foot in s.Program.Feet do
            let body = g.Bodies |> List.find (fun b -> b.Body = foot.Port.Body)
            let choice = g.Feet |> List.find (fun f -> f.Foot = foot.Id)
            match body.Parts |> List.filter (fun p -> p.Id = choice.Part) with
            | [part] when supportedBeam part.Part && body.Parts.Length = 1 ->
                let sourcePose = s.Frames.Head.Poses[body.Body]
                let ports = s.Program.Hinges |> List.collect (fun h -> [h.A;h.B])
                            |> List.filter (fun p -> p.Body = body.Body) |> List.distinct
                if ports.Length >= 2 then
                    let old = (s.Program.Bodies |> List.find (fun b -> b.Id = body.Body)).Sites |> List.find (fun site -> site.Id = foot.Port.Site)
                    let previous = point s foot.Port
                    let anchor = ports |> List.sortBy (fun port -> -Vec3.distance (point s port) previous, port) |> List.head
                    let root = point s anchor
                    let requirements = ports |> List.choose (fun port ->
                        let expected = point s port
                        sockets part |> List.tryFind (fun (_,p) -> near p.X expected.X && near p.Y expected.Y)
                        |> Option.map (fun (socket,_) -> port,expected,socket.Kind))
                    let centroid = ports |> List.map (point s) |> List.reduce Vec3.add |> Vec3.scale (1.0/float ports.Length)
                    let outward = Vec3.subtract previous centroid
                    let length = Vec3.length outward
                    let used = ports |> List.exists (fun p -> p.Site = foot.Port.Site)
                    if not used && requirements.Length = ports.Length && length > 1e-6 then
                      let direction = Vec3.scale (1.0/length) outward
                      let projection p = Vec3.dot (Vec3.subtract p centroid) direction
                      let hingeEdge = ports |> List.map (point s >> projection) |> List.max
                      let anchorKind = requirements |> List.find (fun (port,_,_) -> port = anchor) |> fun (_,_,kind) -> kind
                      for target, tip, operation in templates do
                        for rootSocket in (Catalog.get target).Ports |> List.filter (fun socket -> socket.Kind = anchorKind) do
                          for along in [1.0; -1.0] do
                           for side in (if operation = "bend-foot" then [1.0;-1.0] else [1.0]) do
                            let rotation =
                                { X = Vec3.scale (along*side) part.Rotation.X
                                  Y = Vec3.scale side part.Rotation.Y
                                  Z = Vec3.scale along part.Rotation.Z }
                            let sign = if part.Position.Z < center then -1.0 else 1.0
                            let depth = part.Position.Z + (if part.Part = "32063.dat" then sign*5.0 else 0.0)
                            let replacement = {
                                part with Part = target; Rotation = rotation
                                          Position = Vec3.subtract { root with Z = depth } (Rotation.apply rotation rootSocket.Position)
                            }
                            let matches =
                                requirements |> List.forall (fun (_,expected,kind) ->
                                    sockets replacement |> List.exists (fun (socket,position) ->
                                        socket.Kind = kind && near position.X expected.X && near position.Y expected.Y))
                            let contact = Vec3.add replacement.Position (Rotation.apply rotation tip)
                            let local = Rotation.apply (Rotation.aroundZ -sourcePose.Angle) (Vec3.subtract contact sourcePose.Origin)
                            let planar = { local with Z = 0.0 }
                            let reachesBelowChassis =
                                s.Frames |> List.exists (fun frame ->
                                    (Motion.transform frame.Poses[body.Body] planar).Y+foot.Radius >= chassisLevel)
                            if matches && projection previous >= hingeEdge-1e-6 && projection contact > hingeEdge+1e-6
                               && Vec3.distance planar old.Position > 1e-6 && reachesBelowChassis
                               && (operation <> "lengthen-foot" || projection contact > projection previous+1.0) then
                                let programBodies =
                                    s.Program.Bodies |> List.map (fun b ->
                                        if b.Id <> body.Body then b else
                                            let sites = b.Sites |> List.map (fun site ->
                                                if site.Id = foot.Port.Site then { site with Position = { local with Z = 0.0 } } else site)
                                            { b with Sites = sites })
                                let program = { s.Program with Bodies = programBodies }
                                let genome = {
                                    g with Bodies = g.Bodies |> List.map (fun b ->
                                               if b.Body = body.Body then { b with Parts = [replacement] } else b)
                                           Feet = g.Feet |> List.map (fun f -> if f.Foot = foot.Id then { f with Depth = depth } else f)
                                }
                                yield { Program = program; Genome = genome; Operation = operation
                                        Changes = [($"feet/{foot.Id}/part:{part.Part}->{target}", [0.0], [1.0])
                                                   ($"feet/{foot.Id}/contact", [old.Position.X;old.Position.Y], [local.X;local.Y])] }
            | _ -> ()
    ]

    let gearingEdits (s: ScreenedMotion) (g: WalkerGenome) = [
        let motor = s.Program.Motor.Body
        let outgoing = s.Program.Transmissions |> List.filter (fun t -> t.Input = motor)
        match outgoing with
        | [edge] ->
            let input = g.Bodies |> List.find (fun b -> b.Body = edge.Input)
            let output = g.Bodies |> List.find (fun b -> b.Body = edge.Output)
            let gears body = body.Parts |> List.filter (fun p -> (Catalog.gearTeeth p.Part).IsSome)
            let inputGears = gears input
            let outputGears = gears output
            let mesh =
                match inputGears with
                | [a] -> outputGears |> List.filter (fun b -> near a.Position.Z b.Position.Z)
                         |> List.map (fun b -> a,b)
                | _ -> []
            match mesh with
            | [a,b] when a.Part = "3648b.dat" && b.Part = "3648b.dat" && near edge.Ratio -1.0 ->
                let existingCompound = outputGears.Length > 1
                let rec descendants reached =
                    let next = s.Program.Transmissions |> List.fold (fun ids t ->
                        if Set.contains t.Input ids then Set.add t.Output ids else ids) reached
                    if next = reached then reached else descendants next
                let moved = descendants (Set.singleton edge.Output) |> Set.remove edge.Output
                let extra = { b with Id = b.Id+"--compound"; Position = Vec3.add b.Position (Vec3.create 0 0 20) }
                let bodies = g.Bodies |> List.map (fun body ->
                    if body.Body = input.Body then
                        { body with Parts = body.Parts |> List.map (fun p -> if p.Id = a.Id then { p with Part = "3647.dat" } else p) }
                    elif body.Body = output.Body then
                        { body with Parts = (body.Parts |> List.map (fun p -> if p.Id = b.Id then { p with Part = "3649.dat" } else p))
                                            @ (if existingCompound then [] else [extra]) }
                    elif moved.Contains body.Body && not existingCompound then
                        let parts = body.Parts |> List.map (fun p ->
                            if (Catalog.gearTeeth p.Part).IsSome then shift (Vec3.create 0 0 20) p else p)
                        { body with Parts = parts }
                    else body)
                let ratio = -0.2
                let phase = s.Frames.Head.Poses[edge.Output].Angle-ratio*s.Frames.Head.Poses[edge.Input].Angle
                let transmissions = s.Program.Transmissions |> List.map (fun t ->
                    if t = edge then { t with Ratio = ratio; Phase = phase } else t)
                let program = { s.Program with Transmissions = transmissions }
                yield { Program = program; Genome = { g with Bodies = bodies }; Operation = "gear-reduction"
                        Changes = [($"transmissions/{edge.Input}->{edge.Output}/ratio", [edge.Ratio], [ratio])] }
            | _ -> ()
        | _ -> ()
    ]

    let private branchEditsBase (s: ScreenedMotion) (g: WalkerGenome) = [
        let chassis = s.Program.Bodies |> List.find _.Grounded
        let driven = protectedBodies s.Program |> Set.remove chassis.Id
        let bodyChoices = g.Bodies |> List.map (fun b -> b.Body,b) |> Map.ofList
        let namePrefix = prefix s.Program
        for foot in s.Program.Feet do
            let members = branchMembers s.Program foot.Port.Body
            let outputs = s.Program.Feet |> List.filter (fun f -> members.Contains f.Port.Body)
            if outputs.Length = 1 && not members.IsEmpty then
                let hinges = s.Program.Hinges |> List.filter (fun h -> members.Contains h.A.Body || members.Contains h.B.Body)
                let hingeIds = hinges |> List.map _.Id |> Set.ofList
                let boundaries = hinges |> List.collect (fun h -> [h.A;h.B]) |> List.filter (fun p -> not (members.Contains p.Body)) |> List.distinct
                let inputs = boundaries |> List.filter (fun p -> driven.Contains p.Body) |> List.map _.Body |> List.distinct
                let connections = g.Connections |> List.filter (fun c -> c.Hinges |> List.exists hingeIds.Contains)
                if connections |> List.forall (fun c -> c.Hinges |> List.forall hingeIds.Contains) then
                    let implementing port =
                        let pivot = connections |> List.find (fun c -> c.Hinges |> List.exists (fun id ->
                            let h = hinges |> List.find (fun h -> h.Id = id)
                            h.A = port || h.B = port))
                        bodyChoices[port.Body].Parts |> List.filter (engaged pivot.Placement)
                    let driveParts = boundaries |> List.filter (fun p -> driven.Contains p.Body)
                                     |> List.collect implementing |> List.distinctBy _.Id
                    let removedParts = driveParts |> List.map _.Id |> Set.ofList
                    if s.Program.Feet.Length > 2 then
                        let program = { s.Program with
                                            Bodies = s.Program.Bodies |> List.filter (fun b -> not (members.Contains b.Id))
                                            Hinges = s.Program.Hinges |> List.filter (fun h -> not (hingeIds.Contains h.Id))
                                            Feet = s.Program.Feet |> List.filter (fun f -> f.Id <> foot.Id) }
                        let genome = { g with
                                           Bodies = g.Bodies |> List.filter (fun b -> not (members.Contains b.Body))
                                                    |> List.map (fun b -> { b with Parts = b.Parts |> List.filter (fun p -> not (removedParts.Contains p.Id)) })
                                           Connections = g.Connections |> List.filter (fun c -> not (c.Hinges |> List.exists hingeIds.Contains))
                                           Feet = g.Feet |> List.filter (fun f -> f.Foot <> foot.Id) }
                        yield { Program = program; Genome = genome; Operation = "remove-branch"
                                Changes = [($"feet/{foot.Id}", [1.0], []); ("footCount", [float s.Program.Feet.Length], [float program.Feet.Length])] }
                    match inputs with
                    | [source] when s.Program.Feet.Length < Motion.maximumFeet && s.Program.Bodies.Length+members.Count <= 32 ->
                        let sourcePose = s.Frames.Head.Poses[source]
                        let outputShaft id =
                            id <> s.Program.Motor.Body && (g.Connections |> List.exists (fun c ->
                                List.contains c.Placement.Part Catalog.axleParts
                                && (let lo,hi = Catalog.shaftRange c.Placement.Part in hi-lo>=160.0)
                                && c.Hinges |> List.exists (fun hid ->
                                    let h = s.Program.Hinges |> List.find (fun h -> h.Id = hid)
                                    Set.ofList [h.A.Body;h.B.Body] = Set.ofList [chassis.Id;id])))
                        for target in s.Program.Bodies |> List.filter (fun b -> driven.Contains b.Id && b.Id <> source && outputShaft b.Id) do
                            let targetPose = s.Frames.Head.Poses[target.Id]
                            let delta = Vec3.subtract targetPose.Origin sourcePose.Origin
                            let angle = targetPose.Angle-sourcePose.Angle
                            let incoming id = s.Program.Transmissions |> List.tryFind (fun t -> t.Output = id)
                            let rec rate seen id =
                                if Set.contains id seen then None
                                elif id = s.Program.Motor.Body then Some 1.0
                                else incoming id |> Option.bind (fun t -> rate (Set.add id seen) t.Input |> Option.map ((*) t.Ratio))
                            let compatible = match rate Set.empty source,rate Set.empty target.Id with
                                             | Some a,Some b -> near a b | _ -> false
                            if compatible && near (sin angle) 0.0 && near (cos angle) 1.0 then
                                let collision = driveParts |> List.exists (fun p -> bodyChoices[target.Id].Parts |> List.exists (fun q ->
                                    p.Part = q.Part && Vec3.distance (Vec3.add p.Position delta) q.Position < 1e-6))
                                if not collision then
                                    let rename = renamed namePrefix
                                    let boundary (port: MotionPort) : MotionPort =
                                        if members.Contains port.Body then { port with Body = rename port.Body }
                                        else { Body = (if port.Body = source then target.Id else port.Body); Site = rename port.Site }
                                    let additions id =
                                        boundaries |> List.filter (fun p -> p.Body = id) |> List.map (fun port ->
                                            let world = Vec3.add (point s port) delta
                                            let pose = if id = source then targetPose else s.Frames.Head.Poses[id]
                                            { Id = rename port.Site
                                              Position = Rotation.apply (Rotation.aroundZ -pose.Angle) (Vec3.subtract world pose.Origin) })
                                    let shiftedConnections = connections |> List.map (fun c ->
                                        { Group = rename c.Group; Hinges = c.Hinges |> List.map rename
                                          Placement = { shift delta c.Placement with Id = "pivot-"+rename c.Group } })
                                    let supports =
                                        boundaries |> List.filter (fun p -> p.Body = chassis.Id)
                                        |> List.collect (fun port ->
                                            let original = connections |> List.find (fun c -> c.Hinges |> List.exists (fun id ->
                                                let h = hinges |> List.find (fun h -> h.Id = id)
                                                h.A = port || h.B = port))
                                            let movedPin = shift delta original.Placement
                                            if bodyChoices[chassis.Id].Parts |> List.exists (engaged movedPin) then []
                                            else implementing port |> List.filter (fun p -> supportedBeam p.Part))
                                        |> List.distinctBy _.Id
                                    let unsupported = boundaries |> List.filter (fun p -> p.Body = chassis.Id) |> List.exists (fun port ->
                                        let world = Vec3.add (point s port) delta
                                        let implemented = bodyChoices[chassis.Id].Parts @ (supports |> List.map (shift delta))
                                        not (implemented |> List.exists (fun p -> sockets p |> List.exists (fun (_,q) -> near q.X world.X && near q.Y world.Y))))
                                    if not unsupported then
                                        let mounts = g.Mounts |> List.filter (fun pin -> supports |> List.exists (engaged pin))
                                                     |> List.map (fun pin -> { shift delta pin with Id = rename pin.Id })
                                        let branchBodies = s.Program.Bodies |> List.filter (fun b -> members.Contains b.Id) |> List.map (fun b ->
                                            { b with Id = rename b.Id; Initial = { b.Initial with Origin = Vec3.add b.Initial.Origin delta } })
                                        let program = {
                                            s.Program with
                                                Bodies = (s.Program.Bodies |> List.map (fun b ->
                                                    if b.Id = target.Id then { b with Sites = b.Sites @ additions source }
                                                    elif b.Id = chassis.Id then { b with Sites = b.Sites @ additions chassis.Id }
                                                    else b)) @ branchBodies
                                                Hinges = s.Program.Hinges @ (hinges |> List.map (fun h -> { Id = rename h.Id; A = boundary h.A; B = boundary h.B }))
                                                Feet = s.Program.Feet @ [{ foot with Id = rename foot.Id; Port = boundary foot.Port }]
                                        }
                                        let extraParts ps = ps |> List.map (fun p -> { shift delta p with Id = rename p.Id })
                                        let inheritedBodies = g.Bodies |> List.map (fun b ->
                                            if b.Body = target.Id then { b with Parts = b.Parts @ extraParts driveParts }
                                            elif b.Body = chassis.Id then { b with Parts = b.Parts @ extraParts supports }
                                            else b)
                                        let copiedBodies = g.Bodies |> List.filter (fun b -> members.Contains b.Body) |> List.map (fun b ->
                                            { Body = rename b.Body; Parts = extraParts b.Parts })
                                        let bodies = inheritedBodies @ copiedBodies
                                        let newDriveParts = extraParts driveParts
                                        let pivots = g.Connections |> List.map (fun c ->
                                            if c.Hinges |> List.exists (fun id ->
                                                let h = s.Program.Hinges |> List.find (fun h -> h.Id = id)
                                                Set.ofList [h.A.Body;h.B.Body] = Set.ofList [chassis.Id;target.Id])
                                               && (Catalog.get c.Placement.Part).Connector = Some Axle
                                               && (newDriveParts |> List.exists (engaged c.Placement >> not)) then
                                                { c with Placement = { c.Placement with Part = "3708.dat" } }
                                            else c)
                                        let choice = g.Feet |> List.find (fun f -> f.Foot = foot.Id)
                                        let genome = { g with Bodies = bodies; Connections = pivots @ shiftedConnections
                                                              Mounts = g.Mounts @ mounts
                                                              Feet = g.Feet @ [{ choice with Foot = rename foot.Id; Part = rename choice.Part
                                                                                             AdditionalContacts=choice.AdditionalContacts |> Option.map (List.map (fun c -> {c with Part=rename c.Part})) }] }
                                        yield { Program = program; Genome = genome; Operation = "duplicate-branch"
                                                Changes = [($"branches/{foot.Id}->{rename foot.Id}", [], [delta.X;delta.Y])
                                                           ("footCount", [float s.Program.Feet.Length], [float program.Feet.Length])] }
                    | _ -> ()
    ]

    let private extendDrive (s: ScreenedMotion) (g: WalkerGenome) =
        let shafts = s.Program.Bodies |> List.filter (fun b -> b.Id.StartsWith "shaft-")
        let stations = (shafts.Length+1)/4
        let expected = 2+(stations-1)*4
        let canonical =
            shafts.Length = expected+1
            && ([0..expected] |> List.forall (fun n ->
                shafts |> List.exists (fun b -> b.Id = $"shaft-{n}" && near b.Initial.Origin.X (40.0+float n*60.0) && near b.Initial.Origin.Y 0.0)))
        if not canonical || stations >= 3 || s.Program.Bodies.Length+4 > 30 then None else
            let platform = Platform.build Genome.defaults (stations+1) s.Program.Motor.Phase
            let freshIds = [expected+1..expected+4] |> List.map (fun n -> $"shaft-{n}") |> Set.ofList
            let placement (p: PartInstance) : WalkerPlacement =
                { Id = p.Id; Part = p.Part; Color = p.Color; Position = p.Position; Rotation = p.Rotation }
            let freshParts = platform.Parts |> List.filter (fun p -> p.Body |> Option.exists freshIds.Contains)
            let freshBodies = freshParts |> List.groupBy (fun p -> p.Body.Value) |> List.map (fun (id,ps) ->
                let gear = ps |> List.find (fun p -> (Catalog.gearTeeth p.Part).IsSome)
                let previous = s.Frames.Head.Poses[$"shaft-{expected}"]
                let index = Int32.Parse(id.Substring("shaft-".Length))
                let angle = if (index-expected)%2 = 0 then previous.Angle else Math.PI/24.0-previous.Angle
                Motion.rigid id gear.Position.X gear.Position.Y angle [Motion.site "root" 0 0])
            let freshJoints = platform.Joints |> List.filter (fun j -> freshIds.Contains j.BodyB)
            let groundSites = freshJoints |> List.map (fun j -> Motion.site j.Id j.Position.X j.Position.Y)
            let transmissions = [
                for n in expected+1..expected+4 -> Motion.transmit $"shaft-{n-1}" $"shaft-{n}" -1 (Math.PI/24.0)
            ]
            let program = {
                s.Program with
                    Bodies = (s.Program.Bodies |> List.map (fun b ->
                        if b.Grounded then { b with Sites = b.Sites @ groundSites } else b)) @ freshBodies
                    Hinges = s.Program.Hinges @ (freshJoints |> List.map (fun j ->
                        Motion.couple j.Id (Motion.port j.BodyA j.Id) (Motion.port j.BodyB "root")))
                    Transmissions = s.Program.Transmissions @ transmissions
            }
            let existingParts = g.Bodies |> List.collect _.Parts |> List.map _.Id |> Set.ofList
            let rails = platform.Parts |> List.filter (fun p ->
                p.Body = Some "chassis" && (p.Id.StartsWith "rail-" || p.Id.StartsWith "bridge-") && not (existingParts.Contains p.Id))
            let choices = freshParts |> List.groupBy (fun p -> p.Body.Value) |> List.map (fun (id,ps) ->
                let pose = freshBodies |> List.find (fun b -> b.Id = id) |> _.Initial
                let parts = ps |> List.map (fun p -> { placement p with Rotation = Rotation.aroundZ pose.Angle })
                { Body = id; Parts = parts })
            let pivots = freshJoints |> List.map (fun j ->
                let p = platform.Parts |> List.find (fun p -> p.Id = j.Connector)
                let angle = freshBodies |> List.find (fun b -> b.Id = j.BodyB) |> _.Initial |> _.Angle
                { Group = j.Id; Hinges = [j.Id]; Placement = { placement p with Id = "pivot-"+j.Id; Rotation = Rotation.shaft angle } })
            let genome = {
                g with Bodies = (g.Bodies |> List.map (fun b ->
                           if b.Body = "chassis" then { b with Parts = b.Parts @ (rails |> List.map placement) } else b)) @ choices
                       Connections = g.Connections @ pivots
            }
            Some (program,genome)

    let branchEdits s g =
        let extended = [
            match extendDrive s g with
            | Some (program,genome) ->
                match Realization.screen program 72 with
                | Ok screened ->
                    for edit in branchEditsBase screened genome do
                        if edit.Operation = "duplicate-branch" then
                            yield { edit with Changes = ("drive-station/shafts", [], [4.0])::edit.Changes }
                | Error _ -> ()
            | None -> ()
        ]
        extended @ branchEditsBase s g

    let edits s g = footEdits s g @ gearingEdits s g @ branchEdits s g
