namespace LegoWalkers

open System

/// Motion-first changes to passive spans, including the opposite links of parallelograms.
module WalkerLengths =
    type private Span = {
        Body: string
        Anchor: MotionPort
        Tip: MotionPort
        A: string
        B: string
        Length: float
    }

    let private site (program: MotionProgram) (port: MotionPort) =
        (program.Bodies |> List.find (fun b -> b.Id=port.Body)).Sites
        |> List.find (fun s -> s.Id=port.Site) |> _.Position

    let private distance program (span: Span) =
        Vec3.distance (site program span.Anchor) (site program span.Tip)

    let proposals (s: ScreenedMotion) (g: WalkerGenome) =
        let protectedIds = WalkerStructural.protectedBodies s.Program
        let groups =
            g.Connections |> List.map (fun c ->
                c.Group, (s.Program.Hinges |> List.filter (fun h -> List.contains h.Id c.Hinges)
                          |> List.collect (fun h -> [h.A;h.B]) |> List.distinct))
            |> Map.ofList
        let linked a b =
            let owners id = groups[id] |> List.map _.Body |> Set.ofList
            a<>b && not (Set.intersect (owners a) (owners b)).IsEmpty
        let spans = [
            for body in s.Program.Bodies |> List.sortBy _.Id do
                let physical = g.Bodies |> List.find (fun b -> b.Body=body.Id)
                if not (protectedIds.Contains body.Id) && physical.Parts.Length=1
                   && not (g.Mounts |> List.exists (fun pin -> WalkerStructural.engaged pin physical.Parts.Head)) then
                    let ports =
                        groups |> Map.toList |> List.choose (fun (id,ports) ->
                            ports |> List.tryFind (fun p -> p.Body=body.Id) |> Option.map (fun port -> id,port))
                    for i in 0..ports.Length-2 do
                      for j in i+1..ports.Length-1 do
                        let a,p = ports[i]
                        let b,q = ports[j]
                        let start,finish =
                            if (Vec3.length (site s.Program p),p.Site) <= (Vec3.length (site s.Program q),q.Site)
                            then p,q else q,p
                        let origin = site s.Program start
                        let vector = Vec3.subtract (site s.Program finish) origin
                        let length = Vec3.length vector
                        if length>=20.0-1e-6 && body.Sites |> List.forall (fun point ->
                            Vec3.length (Vec3.cross vector (Vec3.subtract point.Position origin)) < 1e-6) then
                            yield {Body=body.Id;Anchor=start;Tip=finish;A=a;B=b;Length=length}
        ]
        let opposite = [
            for i in 0..spans.Length-2 do
              for j in i+1..spans.Length-1 do
                let a,b = spans[i],spans[j]
                if a.Body<>b.Body && (Set.ofList [a.A;a.B;b.A;b.B] |> Set.count) = 4
                   && abs(a.Length-b.Length)<1e-6
                   && ((linked a.A b.A && linked a.B b.B) || (linked a.A b.B && linked a.B b.A)) then
                    let vector frame (span: Span) =
                        let point port = Motion.position s.Graph frame port |> Result.defaultWith invalidOp
                        Vec3.subtract (point span.Tip) (point span.Anchor)
                    let sign = if Vec3.dot (vector s.Frames.Head a) (vector s.Frames.Head b) < 0.0 then -1.0 else 1.0
                    if s.Frames |> List.forall (fun frame ->
                        Vec3.distance (vector frame a) (Vec3.scale sign (vector frame b)) < 1e-5) then
                        yield a,b
        ]
        let resize (program: MotionProgram) (span: Span) length =
            let origin = site program span.Anchor
            let direction = Vec3.subtract (site s.Program span.Tip) (site s.Program span.Anchor)
                            |> Vec3.scale (1.0/span.Length)
            let target = Vec3.add origin (Vec3.scale length direction)
            let bodies =
                program.Bodies |> List.map (fun b ->
                    if b.Id<>span.Body then b else
                        let sites = b.Sites |> List.map (fun point ->
                            if point.Id=span.Tip.Site then {point with Position=target} else point)
                        {b with Sites=sites})
            {program with Bodies=bodies}
        [
            for span in spans do
              for delta in [-20.0;20.0] do
                if span.Length+delta>=20.0-1e-6 && span.Length+delta<=240.0+1e-6 then
                    let mutable program = resize s.Program span (span.Length+delta)
                    let mutable consistent = true
                    let mutable changed = true
                    let mutable rounds = 0
                    while changed && consistent && rounds<=opposite.Length do
                        changed <- false
                        rounds <- rounds+1
                        for a,b in opposite do
                            let la,lb = distance program a,distance program b
                            if abs(la-lb)>1e-6 then
                                let ca,cb = abs(la-a.Length)>1e-6,abs(lb-b.Length)>1e-6
                                if ca=cb then consistent <- false
                                else
                                    let target,length = if ca then b,la else a,lb
                                    if length<20.0-1e-6 || length>240.0+1e-6 then consistent <- false
                                    else
                                        program <- resize program target length
                                        changed <- true
                    if consistent && not changed then
                        let changes = [
                            for body in program.Bodies do
                              for point in body.Sites do
                                let before = site s.Program (Motion.port body.Id point.Id)
                                if Vec3.distance before point.Position>1e-6 then
                                    yield $"bodies/{body.Id}/sites/{point.Id}/position",
                                          [before.X;before.Y],[point.Position.X;point.Position.Y]
                        ]
                        if not changes.IsEmpty then yield program,changes
        ] |> List.distinct

    let rebuild policy (parent: ScreenedMotion) (child: ScreenedMotion) (g: WalkerGenome) =
        let changed id =
            (parent.Program.Bodies |> List.find (fun b -> b.Id=id)).Sites
            <> (child.Program.Bodies |> List.find (fun b -> b.Id=id)).Sites
        let sockets (part: WalkerPlacement) =
            (Catalog.get part.Part).Ports |> List.map (fun socket ->
                socket,Vec3.add part.Position (Rotation.apply part.Rotation socket.Position))
        let choices = g.Bodies |> List.choose (fun body ->
            if not (changed body.Body) then None else
                if body.Parts.Length<>1 then invalidArg "limb" $"Cannot refit compound limb {body.Body}."
                let source = body.Parts.Head
                let requirements =
                    (child.Program.Bodies |> List.find (fun b -> b.Id=body.Body)).Sites
                    |> List.map (fun point ->
                        let port = Motion.port body.Body point.Id
                        let old = Motion.transform child.Frames.Head.Poses[body.Body] (site parent.Program port)
                        let kind =
                            sockets source |> List.tryFind (fun (_,p) ->
                                abs(p.X-old.X)<1e-5 && abs(p.Y-old.Y)<1e-5)
                            |> Option.map (fst >> _.Kind)
                            |> Option.defaultWith (fun () -> invalidArg "limb" $"No inherited socket for {body.Body}/{point.Id}.")
                        let target = Motion.transform child.Frames.Head.Poses[body.Body] point.Position
                        {target with Z=source.Position.Z},kind)
                Some (body.Body,WalkerConstruction.beamFits source requirements))
        let candidate = WalkerConstruction.fitBeams child g choices
        let pivots = candidate.Connections |> List.map (fun pivot ->
            let affected = WalkerConstruction.members child.Program pivot |> List.exists changed
            if not affected || (Catalog.get pivot.Placement.Part).Connector<>Some FrictionlessPin
               || WalkerConstruction.pivotSupported child.Program candidate pivot then pivot
            else
                [for part in pivot.Placement.Part::policy.PinOrder |> List.distinct do
                    for offset in [0.0;-5.0;5.0;-10.0;10.0] ->
                        {pivot with Placement={pivot.Placement with Part=part;Position=Vec3.add pivot.Placement.Position (Vec3.scale offset pivot.Placement.Rotation.X)}}]
                |> List.tryFind (WalkerConstruction.pivotSupported child.Program candidate)
                |> Option.defaultWith (fun () -> invalidArg "limb" $"No supported smooth pin covers revised pivot {pivot.Group}."))
        {candidate with Connections=pivots}
