namespace LegoWalkers

open System

module WalkerLoops =
    let private near a b = abs (a-b) < 1e-6
    let private position (s: ScreenedMotion) port =
        Motion.position s.Graph s.Frames.Head port |> Result.defaultWith invalidOp
    let private sockets (p: WalkerPlacement) =
        (Catalog.get p.Part).Ports |> List.map (fun socket ->
            socket,Vec3.add p.Position (Rotation.apply p.Rotation socket.Position))
    let private at (point: Vec3) depth = { point with Z = depth }

    let private pin (program: MotionProgram) (g: WalkerGenome) (pivot: WalkerPivotChoice) =
        let members =
            program.Hinges |> List.filter (fun h -> List.contains h.Id pivot.Hinges)
            |> List.collect (fun h -> [h.A.Body;h.B.Body]) |> Set.ofList
        let depths =
            g.Bodies |> List.filter (fun b -> members.Contains b.Body) |> List.collect _.Parts
            |> List.filter (fun p -> sockets p |> List.exists (fun (socket,point) ->
                socket.Kind = RoundHole && near point.X pivot.Placement.Position.X
                && near point.Y pivot.Placement.Position.Y
                // Ignore the opposite chassis rail; the longest supported pin spans 60 LDU.
                && abs (point.Z-pivot.Placement.Position.Z) <= 60.0
                && near (abs (Rotation.apply p.Rotation socket.Axis).Z) 1.0))
            |> List.map _.Position.Z
        if depths.IsEmpty then None
        else
            let center = (List.min depths+List.max depths)/2.0
            [for part in ["3673.dat";"32556.dat"] do
                for offset in [0.0;-5.0;5.0] ->
                    { pivot with Placement = { pivot.Placement with Part = part; Position = at pivot.Placement.Position (center+offset) } }]
            |> List.tryFind (WalkerConstruction.pivotSupported program g)

    let private build (s: ScreenedMotion) (g: WalkerGenome) prefix (foot: MotionFoot)
                      (reference: MotionPort) (root: MotionPort) (tip: MotionPort)
                      (joint: WalkerPivotChoice) (anchor: Vec3) direction depth =
        let fixedBody = s.Program.Bodies |> List.find (fun b -> b.Id = reference.Body)
        let pose = s.Frames.Head.Poses[fixedBody.Id]
        let local = Rotation.apply (Rotation.aroundZ -pose.Angle) (Vec3.subtract anchor pose.Origin)
        let anchorId = prefix+"-anchor"
        let program = {
            s.Program with
                Bodies = s.Program.Bodies |> List.map (fun b ->
                    if b.Id = fixedBody.Id then { b with Sites = b.Sites @ [{Id=anchorId;Position=at local 0.0}] } else b)
        }
        let span = Vec3.subtract anchor (position s reference)
        match Motion.addHingedFoot prefix (reference,Motion.port reference.Body anchorId) (root,tip)
                  ("left","right") (Motion.site "contact" (span.X/2.0) 14.0) foot.Id program with
        | Error _ -> None
        | Ok program ->
            let parallelId,outputId = prefix+"-parallel",prefix+"-output"
            let parallelDepth,outputDepth = depth+direction*20.0,depth+direction*40.0
            let endPoint = position s tip
            // Prefer the thin link to clear the retained coupler in the adjacent layer.
            let fits = [
                for part in ["32063.dat";"32316.dat";"32524.dat";"32525.dat"] do
                    for pos,rot in Catalog.fitPlanarPart part (Vec3.create 0 0 1)
                                       [at anchor parallelDepth; at (Vec3.add endPoint span) parallelDepth] do
                        yield {Id=parallelId+"-part";Part=part;Color=14;Position=pos;Rotation=rot}
            ]
            match fits with
            | [] -> None
            | parallelPart::_ ->
                let sole = {
                    Id=outputId+"-part";Part="3701.dat";Color=2;Rotation=Rotation.identity
                    Position=at (Vec3.add endPoint (Vec3.create (span.X/2.0) -10.0 0)) outputDepth
                }
                let pivot suffix point =
                    { Group=prefix+suffix;Hinges=[prefix+suffix]
                      Placement={Id="pivot-"+prefix+suffix;Part="3673.dat";Color=7
                                 Position=point;Rotation=Rotation.shaft 0.0} }
                let affected = [
                    {joint with Hinges=joint.Hinges @ [prefix+"-driver"]}
                    pivot "-base" (at anchor depth)
                    pivot "-tip" (at (Vec3.add endPoint span) outputDepth)
                ]
                let candidate = {
                    g with MechanismId=Realization.mechanismId program
                           Bodies=g.Bodies @ [{Body=parallelId;Parts=[parallelPart]};{Body=outputId;Parts=[sole]}]
                           Connections=(g.Connections |> List.filter (fun c -> c.Group <> joint.Group)) @ affected
                           Feet=g.Feet |> List.map (fun f ->
                               if f.Foot = foot.Id then {f with Part=sole.Id;Depth=outputDepth} else f)
                }
                let pins = affected |> List.map (pin program candidate)
                if pins |> List.exists Option.isNone then None
                else
                    let pins = pins |> List.choose id
                    let unchanged = candidate.Connections |> List.filter (fun c ->
                        affected |> List.forall (fun a -> a.Group <> c.Group))
                    Some {
                        Program=program
                        Genome={candidate with Connections=unchanged @ pins}
                        Operation="add-hinged-foot"
                        Changes=[
                            $"bodies/{reference.Body}/sites/{anchorId}",[],[local.X;local.Y]
                            $"bodies/{parallelId},{outputId}",[],[2.0]
                            $"hinges/{prefix}",[],[3.0]
                            $"feet/{foot.Id}/port:{foot.Port.Body}.{foot.Port.Site}->{outputId}.contact",[],[span.X;parallelDepth;outputDepth]
                        ]
                    }

    let edits (s: ScreenedMotion) (g: WalkerGenome) =
        let chassis = s.Program.Bodies |> List.find _.Grounded
        let occupied = s.Program.Bodies |> List.map _.Id |> Set.ofList
        let prefix =
            Seq.initInfinite (fun n -> $"hinged-foot-{n}")
            |> Seq.find (fun p -> not (occupied.Contains (p+"-parallel") || occupied.Contains (p+"-output")))
        let protectedIds =
            s.Program.Transmissions |> List.collect (fun t -> [t.Input;t.Output])
            |> Set.ofList |> Set.add s.Program.Motor.Body
        [
          if s.Program.Bodies.Length <= 30 && near s.Frames.Head.Poses[chassis.Id].Angle 0.0 then
            for foot in s.Program.Feet |> List.sortBy _.Id do
              for bearing in s.Program.Hinges |> List.sortBy _.Id do
                let fixedPort,moving = if bearing.A.Body = chassis.Id then bearing.A,bearing.B else bearing.B,bearing.A
                if fixedPort.Body = chassis.Id && not (protectedIds.Contains moving.Body) then
                  let parts = (g.Bodies |> List.find (fun b -> b.Body = moving.Body)).Parts
                  if parts.Length = 1 then
                    for hinge in s.Program.Hinges |> List.sortBy _.Id do
                      let driver,other = if hinge.A.Body = moving.Body then hinge.A,hinge.B else hinge.B,hinge.A
                      if driver.Body = moving.Body && other.Body = foot.Port.Body && driver <> moving then
                        let joint = g.Connections |> List.find (fun c -> List.contains hinge.Id c.Hinges)
                        if joint.Hinges.Length = 1 && (Catalog.get joint.Placement.Part).Connector = Some FrictionlessPin then
                          let origin = position s fixedPort
                          for frame in (g.Bodies |> List.find (fun b -> b.Body = chassis.Id)).Parts |> List.sortBy _.Id do
                            let delta = parts.Head.Position.Z-frame.Position.Z
                            if abs delta > 1e-6 && abs delta <= 25.0 then
                              let anchors =
                                  sockets frame |> List.choose (fun (socket,point) ->
                                      if socket.Kind = RoundHole && near (abs (Rotation.apply frame.Rotation socket.Axis).Z) 1.0
                                         && near point.Y origin.Y && near (abs (point.X-origin.X)) 40.0
                                      then Some (at point 0.0) else None) |> List.distinct
                              for anchor in anchors do
                                let occupied =
                                    (g.Connections |> List.map _.Placement) @ g.Mounts
                                    |> List.exists (fun p -> near p.Position.X anchor.X && near p.Position.Y anchor.Y)
                                if not occupied then
                                    match build s g prefix foot fixedPort moving driver joint anchor (sign delta |> float) parts.Head.Position.Z with
                                    | Some edit -> yield edit
                                    | None -> ()
        ] |> List.distinctBy (fun e -> WalkerRealization.json e.Genome) |> List.truncate 24
