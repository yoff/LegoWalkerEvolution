namespace LegoWalkers

open System

module WalkerLayout =
    let private near a b = abs (a-b) < 1e-6
    let private coordinates (p: Vec3) = [p.X;p.Y;p.Z]
    let private shifted delta (p: WalkerPlacement) = { p with Position = Vec3.add p.Position delta }
    let private chassis (s: ScreenedMotion) (g: WalkerGenome) =
        let id = s.Program.Bodies |> List.find _.Grounded |> _.Id
        g.Bodies |> List.find (fun b -> b.Body = id)

    let batteryEdits (s: ScreenedMotion) (g: WalkerGenome) = [
        let frame = chassis s g
        match frame.Parts |> List.tryFind (fun p -> p.Id = "payload-battery" && p.Part = "58119.dat") with
        | None -> ()
        | Some battery ->
            let supports = frame.Parts |> List.filter (fun p -> p.Id.StartsWith "battery-support-")
            let pins = g.Mounts |> List.filter (fun pin -> WalkerStructural.engaged pin battery)
            let top = int (Math.Round(-battery.Position.Y/24.0))
            let column plane = supports |> List.filter (fun p -> near p.Position.Z plane) |> List.sortByDescending _.Position.Y
            let supported =
                pins.Length = 4 && top >= 3 && top <= 6 && near battery.Position.Y (-24.0*float top)
                && battery.Rotation = Rotation.identity && supports.Length = 2*(top-1)
                && [0.0;120.0] |> List.forall (fun plane ->
                    let ps = column plane
                    ps.Length = top-1 && ps |> List.mapi (fun i p ->
                        p.Part = "3703.dat" && p.Rotation = Rotation.identity
                        && near p.Position.X battery.Position.X && near p.Position.Y (-10.0-24.0*float(i+2))) |> List.forall id)
            if supported then
                let ids = supports |> List.map _.Id |> Set.ofList
                let pinIds = pins |> List.map _.Id |> Set.ofList
                let baseRails = frame.Parts |> List.filter (fun p -> p.Part = "3703.dat" && near p.Position.Y -34.0)
                for dx,level in [-20.0,top;20.0,top;-40.0,top;40.0,top;0.0,top-1;0.0,top+1] do
                    let position = { battery.Position with X = battery.Position.X+dx; Y = -24.0*float level }
                    let anchored = [0.0;120.0] |> List.forall (fun plane ->
                        baseRails |> List.exists (fun p ->
                            near p.Position.Z plane && abs (position.X-p.Position.X) <= 140.0))
                    if level >= 3 && level <= 6 && anchored then
                        let delta = Vec3.subtract position battery.Position
                        let columns = [
                            for plane in [0.0;120.0] do
                                let prototype = (column plane).Head
                                for n in 2..level do
                                    yield { prototype with Id = $"battery-support-{plane}-{n}"
                                                           Position = Vec3.create position.X (-10.0-24.0*float n) plane }
                        ]
                        let parts =
                            frame.Parts |> List.filter (fun p -> not (ids.Contains p.Id))
                            |> List.map (fun p -> if p.Id = battery.Id then { p with Position = position } else p)
                        let genome = {
                            g with
                                Bodies = g.Bodies |> List.map (fun b ->
                                    if b.Body = frame.Body then { b with Parts = parts @ columns } else b)
                                Mounts = g.Mounts |> List.map (fun p -> if pinIds.Contains p.Id then shifted delta p else p)
                        }
                        yield { Program = s.Program; Genome = genome; Operation = "battery-layout"
                                Changes = [("hardware/battery/position",coordinates battery.Position,coordinates position)
                                           ("hardware/battery/support-bricks",[float supports.Length],[float columns.Length])] }
    
    ]

    let receiverEdits (s: ScreenedMotion) (g: WalkerGenome) = [
        let frame = chassis s g
        match frame.Parts |> List.tryFind (fun p -> p.Id = "payload-receiver" && p.Part = "58123.dat") with
        | None -> ()
        | Some receiver ->
            let pins = g.Mounts |> List.filter (fun pin -> WalkerStructural.engaged pin receiver)
            let ports = (Catalog.get receiver.Part).Ports |> List.map _.Position
            let supported =
                pins.Length = 2 && pins |> List.forall (fun pin -> pin.Part = "6558.dat")
                && receiver.Rotation.Y = Vec3.create 0 1 0
                && near (abs receiver.Rotation.Z.Z) 1.0
                && ports |> List.forall (fun port ->
                    let point = Vec3.add receiver.Position (Rotation.apply receiver.Rotation port)
                    pins |> List.exists (fun pin -> Vec3.distance point pin.Position < 1e-6))
            if supported then
                let anchored = frame.Parts |> List.filter (fun p -> p.Id <> receiver.Id)
                let holes =
                    anchored |> List.collect (fun part ->
                        (Catalog.get part.Part).Ports |> List.choose (fun port ->
                            let point = Vec3.add part.Position (Rotation.apply part.Rotation port.Position)
                            let axis = Rotation.apply part.Rotation port.Axis
                            if port.Kind = RoundHole && near (abs axis.Z) 1.0 && near point.Y (receiver.Position.Y+10.0) then
                                Some point
                            else None)) |> List.distinct
                let pinIds = pins |> List.map _.Id |> Set.ofList
                let occupied = (g.Mounts |> List.filter (fun pin -> not (pinIds.Contains pin.Id)))
                               @ (g.Connections |> List.map _.Placement)
                if not holes.IsEmpty then
                    for rotation in [receiver.Rotation
                                     { receiver.Rotation with X = Vec3.scale -1.0 receiver.Rotation.X
                                                              Z = Vec3.scale -1.0 receiver.Rotation.Z }] do
                        let plane = (if rotation.Z.Z < 0.0 then holes |> List.minBy _.Z else holes |> List.maxBy _.Z).Z
                        let positions =
                            holes |> List.filter (fun point -> near point.Z plane)
                            |> List.map (fun point ->
                                Vec3.subtract (Vec3.add point (Vec3.scale 20.0 rotation.Z)) (Rotation.apply rotation ports.Head))
                            |> List.distinct
                            |> List.filter (fun position -> Vec3.distance position receiver.Position > 1e-6 || rotation <> receiver.Rotation)
                            |> List.sortBy (fun position -> abs (position.X-receiver.Position.X),position.X,position.Z)
                        let edits = positions |> List.choose (fun position ->
                            let rotate vector =
                                Vec3.create (Vec3.dot vector receiver.Rotation.X) (Vec3.dot vector receiver.Rotation.Y) (Vec3.dot vector receiver.Rotation.Z)
                                |> Rotation.apply rotation
                            let transform (pin: WalkerPlacement) =
                                { pin with Position = Vec3.add position (rotate (Vec3.subtract pin.Position receiver.Position))
                                           Rotation = { X = rotate pin.Rotation.X; Y = rotate pin.Rotation.Y; Z = rotate pin.Rotation.Z } }
                            let moved = pins |> List.map transform
                            let supported =
                                ports |> List.forall (fun port ->
                                    let anchor = Vec3.subtract (Vec3.add position (Rotation.apply rotation port)) (Vec3.scale 20.0 rotation.Z)
                                    holes |> List.exists (fun point -> Vec3.distance anchor point < 1e-6))
                            let clear = moved |> List.forall (fun pin ->
                                occupied |> List.forall (fun other ->
                                    not (Catalog.shaftsOverlap pin.Part pin.Position pin.Rotation other.Part other.Position other.Rotation)))
                            if not supported || not clear then None
                            else
                                let parts = frame.Parts |> List.map (fun part ->
                                    if part.Id = receiver.Id then { part with Position = position; Rotation = rotation } else part)
                                let genome = {
                                    g with
                                        Bodies = g.Bodies |> List.map (fun body ->
                                            if body.Body = frame.Body then { body with Parts = parts } else body)
                                        Mounts = g.Mounts |> List.map (fun pin -> if pinIds.Contains pin.Id then transform pin else pin)
                                }
                                Some { Program = s.Program; Genome = genome; Operation = "receiver-layout"
                                       Changes = [("hardware/receiver/position",coordinates receiver.Position,coordinates position)
                                                  ("hardware/receiver/facing",coordinates receiver.Rotation.Z,coordinates rotation.Z)
                                                  ("hardware/receiver/mounts",pins |> List.collect (fun p -> coordinates p.Position),
                                                                              moved |> List.collect (fun p -> coordinates p.Position))] })
                        yield! edits |> List.truncate 12
    ]

    let private reroot (program: MotionProgram) root =
        let rec visit visited todo edges =
            match todo with
            | [] -> if List.length edges = program.Transmissions.Length then Some edges else None
            | current::rest ->
                let next =
                    program.Transmissions |> List.choose (fun t ->
                        if t.Input = current && not (Set.contains t.Output visited) then Some t
                        elif t.Output = current && not (Set.contains t.Input visited) then
                            Some { Input = t.Output; Output = t.Input; Ratio = 1.0/t.Ratio; Phase = -t.Phase/t.Ratio }
                        else None)
                let reached = next |> List.fold (fun ids t -> Set.add t.Output ids) visited
                visit reached (rest @ (next |> List.map _.Output)) (edges @ next)
        visit (Set.singleton root) [root] []

    let motorEdits (s: ScreenedMotion) (g: WalkerGenome) = [
        let frame = chassis s g
        match frame.Parts |> List.tryFind (fun p -> p.Id = g.MotorPart && p.Part = "58121.dat") with
        | None -> ()
        | Some motor ->
            let pins = g.Mounts |> List.filter (fun pin -> WalkerStructural.engaged pin motor)
            let supports = frame.Parts |> List.filter (fun p ->
                p.Part = "32523.dat" && pins |> List.forall (fun pin -> WalkerStructural.engaged pin p))
            if pins.Length = 2 && supports.Length = 1 then
                let movingIds = (motor::supports) |> List.map _.Id |> Set.ofList |> Set.add "motor-bracket"
                let bracketPins = g.Mounts |> List.filter (fun pin ->
                    WalkerStructural.engaged pin supports.Head || pin.Id.StartsWith "motor-brace-")
                let pinIds = bracketPins |> List.map _.Id |> Set.ofList
                let anchoredParts = frame.Parts |> List.filter (fun p -> not (movingIds.Contains p.Id))
                let rec rate seen id =
                    if Set.contains id seen then None
                    elif id = s.Program.Motor.Body then Some 1.0
                    else
                        s.Program.Transmissions |> List.tryFind (fun t -> t.Output = id)
                        |> Option.bind (fun t -> rate (Set.add id seen) t.Input |> Option.map ((*) t.Ratio))
                let driven = s.Program.Motor.Body::(s.Program.Transmissions |> List.collect (fun t -> [t.Input;t.Output])) |> List.distinct
                let footCenter =
                    s.Program.Feet |> List.map (fun f ->
                        Motion.position s.Graph s.Frames.Head f.Port |> Result.defaultWith invalidOp)
                    |> List.averageBy _.X
                let candidates = driven |> List.sortBy (fun id -> abs (s.Frames.Head.Poses[id].Origin.X-footCenter),id)
                for id in candidates do
                    let body = g.Bodies |> List.find (fun b -> b.Body = id)
                    let free = body.Parts |> List.forall (fun p ->
                        (Catalog.gearTeeth p.Part).IsSome || List.contains p.Part ["3713.dat";"4265c.dat"])
                    let bearings = g.Connections |> List.filter (fun c ->
                        (Catalog.get c.Placement.Part).Connector = Some Axle
                        && c.Hinges |> List.exists (fun name ->
                            let h = s.Program.Hinges |> List.find (fun h -> h.Id = name)
                            Set.ofList [h.A.Body;h.B.Body] = Set.ofList [frame.Body;id]))
                    match rate Set.empty id, bearings, reroot s.Program id with
                    | Some speed,[bearing],Some transmissions when near speed 1.0 && free ->
                        for face in [1.0;-1.0] do
                            let pose = s.Frames.Head.Poses[id]
                            let position = { pose.Origin with Z = (if face > 0.0 then 150.0 else -30.0) }
                            let rotation = { X = Vec3.create face 0 0; Y = Vec3.create 0 1 0; Z = Vec3.create 0 0 face }
                            let rotate vector =
                                Vec3.create (Vec3.dot vector motor.Rotation.X) (Vec3.dot vector motor.Rotation.Y) (Vec3.dot vector motor.Rotation.Z)
                                |> Rotation.apply rotation
                            let transform (p: WalkerPlacement) =
                                { p with Position = Vec3.add position (rotate (Vec3.subtract p.Position motor.Position))
                                         Rotation = { X = rotate p.Rotation.X; Y = rotate p.Rotation.Y; Z = rotate p.Rotation.Z } }
                            let movedPins = pins |> List.map (fun p ->
                                let moved = transform p
                                { moved with Position = { moved.Position with Z = position.Z-20.0*face } })
                            let connections = g.Connections |> List.map (fun c ->
                                if c.Group = bearing.Group then
                                    { c with Placement = { c.Placement with Part = "3708.dat"; Position = { c.Placement.Position with Z = 60.0 } } }
                                else c)
                            let occupied =
                                (g.Mounts |> List.filter (fun p -> not (pinIds.Contains p.Id)))
                                @ (connections |> List.map _.Placement) @ movedPins
                            let braces = [
                                for sign,label in [-1.0,"left";1.0,"right"] do
                                    let available =
                                        [80.0;60.0;100.0;40.0] |> List.tryPick (fun distance ->
                                            let pin = {
                                                pins.Head with
                                                    Id = $"motor-brace-{label}"; Part = "4459.dat"
                                                    Position = Vec3.add position (Rotation.apply rotation (Vec3.create (sign*distance) 0 -40))
                                                    Rotation = { X = rotation.Z; Y = rotation.Y; Z = Vec3.create -face 0 0 }
                                            }
                                            if anchoredParts |> List.exists (WalkerStructural.engaged pin)
                                               && occupied |> List.forall (fun existing ->
                                                   not (Catalog.shaftsOverlap pin.Part pin.Position pin.Rotation
                                                                                  existing.Part existing.Position existing.Rotation)) then Some pin
                                            else None)
                                    match available with Some pin -> yield pin | None -> ()
                            ]
                            let anchored = braces |> List.forall (fun pin -> anchoredParts |> List.exists (WalkerStructural.engaged pin))
                            if braces.Length = 2 && anchored && (Vec3.distance position motor.Position > 1e-6 || rotation <> motor.Rotation) then
                                let bracket = { transform supports.Head with
                                                    Id = "motor-bracket"; Part = "32525.dat"
                                                    Position = Vec3.add position (Rotation.apply rotation (Vec3.create 0 0 -50)) }
                                let parts = (frame.Parts |> List.filter (fun p -> p.Id <> bracket.Id)
                                             |> List.map (fun p -> if movingIds.Contains p.Id then transform p else p)) @ [bracket]
                                let program = { s.Program with Motor = { Body = id; Phase = pose.Angle }; Transmissions = transmissions }
                                let genome = {
                                    g with
                                        Bodies = g.Bodies |> List.map (fun b ->
                                            if b.Body = frame.Body then { b with Parts = parts }
                                            else
                                                let nearSide = if face > 0.0 then 100.0 else 20.0
                                                let moved = b.Parts |> List.map (fun p ->
                                                    if p.Part = "3713.dat" && near p.Position.Z nearSide
                                                       && near p.Position.Y position.Y && abs (p.Position.X-position.X) <= 100.0 then
                                                        { p with Position = { p.Position with Z = nearSide-20.0*face } }
                                                    else p)
                                                { b with Parts = moved })
                                        Mounts = (g.Mounts |> List.filter (fun p -> not (pinIds.Contains p.Id)))
                                                 @ movedPins @ braces
                                        Connections = connections
                                }
                                yield { Program = program; Genome = genome; Operation = "motor-layout"
                                        Changes = [($"hardware/motor/shaft:{s.Program.Motor.Body}->{id}",[],[1.0])
                                                   ("hardware/motor/position",coordinates motor.Position,coordinates position)] }
                    | _ -> ()
    ]

    let edits s g = batteryEdits s g @ receiverEdits s g @ motorEdits s g
