namespace LegoWalkers

open System

/// Bounded replacement of an independent passive leg, retaining its driven shaft and chassis.
module WalkerLevelFeet =
    let private near a b = abs (a-b) < 1e-6
    let private at depth (p: Vec3) = {p with Z=depth}
    let private sockets (part: WalkerPlacement) =
        (Catalog.get part.Part).Ports |> List.choose (fun socket ->
            let axis = Rotation.apply part.Rotation socket.Axis
            if socket.Kind = RoundHole && near (abs axis.Z) 1.0 then
                Some (Vec3.add part.Position (Rotation.apply part.Rotation socket.Position))
            else None)
    let private point (s: ScreenedMotion) port =
        Motion.position s.Graph s.Frames.Head port |> Result.defaultWith invalidOp
    let private supports part =
        let existing = sockets part
        let planar = existing |> List.distinct |> fun points ->
            points.Length=existing.Length && points |> List.forall (fun p -> near p.Z points.Head.Z)
        let stocks = if planar then ["32524.dat";"32525.dat"] else []
        let expanded = stocks |> List.collect (fun stock ->
            Catalog.fitPlanarPart stock (Vec3.create 0 0 1) existing
            |> List.map (fun (position,rotation) -> {part with Part=stock;Position=position;Rotation=rotation})
            |> List.filter (fun candidate ->
                let points = sockets candidate
                points.Length>existing.Length
                && (points |> List.maxBy _.Y).Y <= (existing |> List.maxBy _.Y).Y+1e-6))
        expanded @ [part] |> List.distinct
    let private rejection code detail =
        {Gate="lego-realizability";Outcome="reject";Code=code;Diagnostics=[detail]}

    let private outwardL (s: ScreenedMotion) prefix =
        let port suffix site = Motion.port (prefix+suffix) site
        s.Frames |> List.forall (fun frame ->
            let position p = Motion.position s.Graph frame p |> Result.defaultWith invalidOp
            let corner = position (port "-upper-output" "corner")
            let toe = position (port "-upper-output" "toe")
            let anchor = position (port "-leg-a" "root")
            Vec3.dot (Vec3.subtract toe corner) (Vec3.subtract corner anchor)>1e-6)

    let private intendedL (s: ScreenedMotion) prefix (part: WalkerPlacement) =
        let world local = Vec3.add part.Position (Rotation.apply part.Rotation local) |> at 0.0
        let corner = point s (Motion.port (prefix+"-upper-output") "corner") |> at 0.0
        let toe = point s (Motion.port (prefix+"-upper-output") "toe") |> at 0.0
        let longTip = world (Vec3.create 0 0 0)
        part.Part="32526.dat"
        && Vec3.distance (world (Vec3.create 0 0 80)) corner<1e-6
        && Vec3.distance (world (Vec3.create 40 0 80)) toe<1e-6
        && near longTip.X corner.X && longTip.Y<corner.Y-60.0
        && outwardL s prefix

    let private prefixFor (s: ScreenedMotion) foot =
        let occupied =
            (s.Program.Bodies |> List.map _.Id)
            @ (s.Program.Bodies |> List.collect _.Sites |> List.map _.Id)
            @ (s.Program.Hinges |> List.map _.Id)
        Seq.initInfinite (fun n -> if n=0 then $"level-foot-{foot}" else $"level-foot-{foot}-replacement-{n}")
        |> Seq.find (fun prefix -> occupied |> List.forall (fun id -> id<>prefix && not (id.StartsWith(prefix+"-"))))

    let private mechanism prefix (input: MotionBody) inputSite lower upper toe branch foot =
        let root = input.Initial.Origin
        let tip = input.Sites |> List.find (fun site -> site.Id=inputSite) |> _.Position
        let crankAngle = input.Initial.Angle+atan2 tip.Y tip.X
        let groundId, inputId = prefix+"-ground",prefix+"-input"
        let ground = Motion.ground groundId [
            {Id="drive";Position=root};{Id="lower";Position=at 0.0 lower};{Id="upper";Position=at 0.0 upper}]
        let baseProgram = {
            Bodies=[ground;{input with Id=inputId;Initial={input.Initial with Angle=crankAngle};
                                      Sites=[Motion.site "root" 0 0;Motion.site "tip" (Vec3.length tip) 0]}]
            Hinges=[Motion.couple (prefix+"-bearing") (Motion.port groundId "drive") (Motion.port inputId "root")]
            Motor={Body=inputId;Phase=crankAngle};Transmissions=[]
            Feet=[Motion.foot foot inputId "tip" 4.0]
        }
        Motion.addDyad (prefix+"-leg") (Motion.port groundId "lower") (Motion.port inputId "tip") 80.0 80.0 branch baseProgram
        |> Result.bind (fun program ->
            let driven = prefix+"-leg-b"
            let program = {
                program with
                    Bodies=program.Bodies |> List.map (fun body ->
                        if body.Id=driven then {body with Sites=body.Sites @ [Motion.site "bottom" -120 0]} else body)
                    Feet=[Motion.foot foot driven "bottom" 4.0]
            }
            Motion.addParallelogram (prefix+"-upper")
                (Motion.port groundId "lower",Motion.port groundId "upper")
                (Motion.port (prefix+"-leg-a") "root",Motion.port (prefix+"-leg-a") "tip")
                ("corner","top") [Motion.site "toe" toe 0] program
            |> Result.bind (Motion.addHingedFoot (prefix+"-sole")
                (Motion.port (prefix+"-upper-output") "corner",Motion.port (prefix+"-upper-output") "toe")
                (Motion.port driven "tip",Motion.port driven "bottom")
                ("left","right") (Motion.site "contact" (toe/2.0) 14.0) foot))
        |> Result.mapError (rejection "unsupported-level-foot-loop")

    let private graft (s: ScreenedMotion) (g: WalkerGenome) (foot: MotionFoot) (members: Set<string>) (support: WalkerPlacement)
                      (inputPort: MotionPort) (inputPart: WalkerPlacement) prefix (template: ScreenedMotion) (fixture: RealizedMotion) =
        let ground = s.Program.Bodies |> List.find _.Grounded
        let input = template.Program.Motor.Body
        let templateGround = template.Program.Bodies |> List.find _.Grounded
        let inputPlacement = fixture.Assembly.Parts |> List.find (fun part -> part.Body=Some input)
        let depth = inputPart.Position.Z-inputPlacement.Position.Z
        let owner id = if id=input then inputPort.Body elif id=templateGround.Id then ground.Id else id
        let port (p: MotionPort) =
            if p.Body=input then inputPort else {p with Body=owner p.Body;Site=if p.Body=templateGround.Id then prefix+"-"+p.Site else p.Site}
        let newHinges = template.Program.Hinges |> List.filter (fun h -> h.Id<>prefix+"-bearing")
                       |> List.map (fun h -> {h with A=port h.A;B=port h.B})
        let program = {
            s.Program with
                Bodies=(s.Program.Bodies |> List.filter (fun b -> not (Set.contains b.Id members))
                        |> List.map (fun b ->
                            if b.Grounded then
                                {b with Sites=b.Sites @ (templateGround.Sites |> List.filter (fun site -> site.Id<>"drive")
                                                       |> List.map (fun site -> {site with Id=prefix+"-"+site.Id}))}
                            else {b with Initial=s.Frames.Head.Poses[b.Id]}))
                       @ (template.Program.Bodies |> List.filter (fun b -> not b.Grounded && b.Id<>input))
                Hinges=(s.Program.Hinges |> List.filter (fun h -> not (Set.contains h.A.Body members || Set.contains h.B.Body members))) @ newHinges
                Feet=s.Program.Feet |> List.map (fun f ->
                    if f.Id=foot.Id then {f with Port=template.Program.Feet.Head.Port} else f)
        }
        let placement (part: PartInstance) : WalkerPlacement =
            {Id=prefix+"-"+part.Id;Part=part.Part;Color=part.Color
             Position=at (part.Position.Z+depth) part.Position;Rotation=part.Rotation}
        let movingPlacement part =
            let p = placement part
            {p with Position=at (p.Position.Z+float fixture.Genome.Side*20.0) p.Position}
        let passive = fixture.Assembly.Parts |> List.choose (fun part ->
            part.Body |> Option.bind (fun id ->
                if id<>input && id<>templateGround.Id then Some {Body=id;Parts=[movingPlacement part]} else None))
        let shiftedCrank = {inputPart with Position=at (inputPart.Position.Z+float fixture.Genome.Side*10.0) inputPart.Position}
        let retainedHinges = program.Hinges |> List.map _.Id |> Set.ofList
        let connections = [
            yield! g.Connections |> List.choose (fun c ->
                let hinges = c.Hinges |> List.filter retainedHinges.Contains
                if hinges.IsEmpty then None else Some {c with Hinges=hinges})
            for group,hinges,_ in fixture.Groups do
                if group<>prefix+"-bearing" then
                    let part = fixture.Assembly.Parts |> List.find (fun p -> p.Id="pivot-"+group)
                    yield {Group=group;Hinges=hinges;Placement={placement part with Id="pivot-"+group}}
        ]
        let sole = fixture.Assembly.Parts |> List.find (fun p -> p.Body=Some template.Program.Feet.Head.Port.Body)
        let mutable candidate = {
            g with MechanismId=Realization.mechanismId program
                   Bodies=(g.Bodies |> List.filter (fun b -> not (Set.contains b.Body members))
                           |> List.map (fun b ->
                               if b.Body=inputPort.Body then
                                   {b with Parts=b.Parts |> List.map (fun p -> if p.Id=inputPart.Id then shiftedCrank else p)}
                               else b)) @ passive
                   Connections=connections
                   Feet=g.Feet |> List.map (fun f ->
                       if f.Foot=foot.Id then {f with Part=(movingPlacement sole).Id;Depth=(movingPlacement sole).Position.Z} else f)
        }
        let mutable failure = None
        let supportedAxles = candidate.Connections |> List.map (fun pivot ->
            if (Catalog.get pivot.Placement.Part).Connector=Some Axle && WalkerStructural.engaged pivot.Placement inputPart then
                let choices = pivot.Placement.Part :: Catalog.axleParts |> List.distinct
                              |> List.sortBy (fun part ->
                                  let lo,hi = Catalog.shaftRange part
                                  hi-lo, (if part=pivot.Placement.Part then 0 else 1))
                let requiredParts = g.Bodies |> List.filter (fun b -> not (members.Contains b.Body)) |> List.collect _.Parts
                                    |> List.filter (WalkerStructural.engaged pivot.Placement) |> List.map _.Id |> Set.ofList
                let walls = candidate.Bodies |> List.collect _.Parts |> List.filter (fun p -> requiredParts.Contains p.Id)
                            |> List.collect (WalkerStructural.engagedPorts pivot.Placement) |> List.collect (fun (_,point) ->
                                let along = Vec3.dot (Vec3.subtract point pivot.Placement.Position) pivot.Placement.Rotation.X
                                [along-10.0;along+10.0])
                let options = choices |> List.collect (fun part ->
                    let lo,hi = Catalog.shaftRange part
                    let offset = if fixture.Genome.Side>0 then List.min walls-lo else List.max walls-hi
                    let shifted = Vec3.add pivot.Placement.Position (Vec3.scale offset pivot.Placement.Rotation.X)
                    let positions = if part=pivot.Placement.Part then [pivot.Placement.Position;shifted] else
                                        [shifted;pivot.Placement.Position]
                    positions |> List.map (fun position -> {pivot with Placement={pivot.Placement with Part=part;Position=position}}))
                let supported (choice: WalkerPivotChoice) =
                    let pin = choice.Placement
                    let lo,hi = Catalog.shaftRange pin.Part
                    let contacts = candidate.Bodies |> List.filter (fun b -> b.Parts |> List.exists (WalkerStructural.engaged pin))
                    let expected = WalkerConstruction.members program choice |> Set.ofList
                    let reachedParts = contacts |> List.collect _.Parts |> List.filter (WalkerStructural.engaged pin)
                                       |> List.map _.Id |> Set.ofList
                    let covered = contacts |> List.collect _.Parts |> List.filter (WalkerStructural.engaged pin)
                                  |> List.forall (WalkerConstruction.socketWallCovered pin)
                    Set.isSubset requiredParts reachedParts
                    && Set.ofList (contacts |> List.map _.Body)=expected && covered
                match options |> List.tryFind supported with
                | Some supported -> supported
                | None ->
                    failure <- Some (rejection "level-foot-input-axle-span" $"No supported stock axle covers the shifted crank at {inputPart.Id}.")
                    pivot
            else pivot)
        candidate <- {candidate with Connections=supportedAxles}
        let refitted = [
            for pivot in candidate.Connections do
                if not (pivot.Group.StartsWith(prefix)) then yield pivot
                else
                    let members = program.Hinges |> List.filter (fun h -> List.contains h.Id pivot.Hinges)
                                  |> List.collect (fun h -> [h.A.Body;h.B.Body]) |> Set.ofList
                    let depths = candidate.Bodies |> List.filter (fun b -> members.Contains b.Body)
                                 |> List.choose (fun body ->
                                     body.Parts |> List.collect (fun part -> sockets part |> List.map (fun p -> part,p))
                                     |> List.filter (fun (_,p) ->
                                         near p.X pivot.Placement.Position.X && near p.Y pivot.Placement.Position.Y
                                         && abs(p.Z-pivot.Placement.Position.Z)<=100.0)
                                     |> List.sortBy (fun (part,p) ->
                                         (if body.Body=ground.Id && part.Id=support.Id then 0 else 1),
                                         abs(p.Z-pivot.Placement.Position.Z))
                                     |> List.tryHead |> Option.map (snd >> _.Z))
                    let choices = [
                        if not depths.IsEmpty then
                            let center = (List.min depths+List.max depths)/2.0
                            for part in ["3673.dat";"32556.dat"] do
                                for offset in [float fixture.Genome.Side*5.0;0.0;float fixture.Genome.Side*(-5.0)] ->
                                    {pivot with Placement={pivot.Placement with Part=part;Position=at (center+offset) pivot.Placement.Position}}
                    ]
                    match choices |> List.tryFind (WalkerConstruction.pivotSupported program candidate) with
                    | Some chosen -> yield chosen
                    | None ->
                        failure <- Some (rejection "level-foot-pivot-span" $"No supported smooth pin covers every declared member at {pivot.Group}.")
                        yield pivot
        ]
        candidate <- {candidate with Connections=refitted}
        for pivot in refitted |> List.filter (fun c -> c.Group.StartsWith(prefix)) do
            if candidate.Mounts |> List.exists (fun pin ->
                Catalog.shaftsOverlap pivot.Placement.Part pivot.Placement.Position pivot.Placement.Rotation pin.Part pin.Position pin.Rotation) then
                failure <- Some (rejection "level-foot-occupied-pivot" $"An existing rigid mount occupies {pivot.Group}.")
        match failure with
        | Some error -> Error error
        | None ->
            Realization.screen program 72 |> Result.bind (fun child ->
                WalkerRealization.compile child candidate |> Result.map (fun _ ->
                    let removed = String.concat "," (Set.toList members)
                    let output = program.Feet |> List.find (fun f -> f.Id=foot.Id) |> _.Port.Body
                    let anchor = templateGround.Sites |> List.find (fun site -> site.Id="lower") |> _.Position
                    let axleChanges = supportedAxles |> List.choose (fun c ->
                        g.Connections |> List.tryFind (fun old -> old.Group=c.Group && old.Placement<>c.Placement)
                        |> Option.map (fun old ->
                            $"connections/{c.Group}/axle:{old.Placement.Part}->{c.Placement.Part}",
                            [old.Placement.Position.Z],[c.Placement.Position.Z]))
                    {Program=program;Genome=candidate;Operation="replace-double-parallelogram-foot"
                     Changes=[$"feet/{foot.Id}/level-loop",[],[80.0;80.0;200.0]
                              $"parts/{inputPart.Id}/depth",[inputPart.Position.Z],[shiftedCrank.Position.Z]
                              $"feet/{foot.Id}/anchor",[],[anchor.X;anchor.Y;float fixture.Genome.Side]
                              $"bodies/{removed}",[float members.Count],[6.0]
                              $"feet/{foot.Id}/port:{foot.Port.Body}.{foot.Port.Site}->{output}.contact",[],[1.0]] @ axleChanges}))

    let attemptsWithLimit maxPerFoot (s: ScreenedMotion) (g: WalkerGenome) =
        if maxPerFoot<1 || maxPerFoot>32 then invalidArg "maxPerFoot" "Construction choices per foot must be between 1 and 32."
        let protectedIds = WalkerStructural.protectedBodies s.Program
        let ground = s.Program.Bodies |> List.find _.Grounded
        let groundParts = (g.Bodies |> List.find (fun b -> b.Body=ground.Id)).Parts
        [
          for foot in s.Program.Feet |> List.sortBy _.Id do
            let mutable accepted = 0
            let members = WalkerStructural.branchMembers s.Program foot.Port.Body
            let others = s.Program.Feet |> List.exists (fun f -> f.Id<>foot.Id && members.Contains f.Port.Body)
            let molded = g.Bodies |> List.exists (fun b -> members.Contains b.Body && b.Parts |> List.exists (fun p -> p.Part="32526.dat"))
            let intended =
                if foot.Port.Body.EndsWith("-sole-output") then
                    let prefix = foot.Port.Body.Substring(0,foot.Port.Body.Length-"-sole-output".Length)
                    if s.Program.Bodies |> List.exists (fun b -> b.Id=prefix+"-leg-a") then
                        g.Bodies |> List.filter (fun b -> b.Body=prefix+"-upper-output")
                        |> List.collect _.Parts |> List.exists (intendedL s prefix)
                    else true
                else true
            let alreadyLevel = molded && s.Frames |> List.forall (fun frame ->
                abs (atan2 (sin frame.Poses[foot.Port.Body].Angle) (cos frame.Poses[foot.Port.Body].Angle))<1e-6)
                               && intended
            let ownedMount = g.Bodies |> List.filter (fun b -> members.Contains b.Body) |> List.collect _.Parts
                             |> List.exists (fun part -> g.Mounts |> List.exists (fun pin -> WalkerStructural.engaged pin part))
            if not others && not alreadyLevel && not ownedMount && s.Program.Bodies.Length-members.Count+6<=32 then
              let edges = s.Program.Hinges |> List.choose (fun h ->
                  if members.Contains h.A.Body && protectedIds.Contains h.B.Body then Some h.B
                  elif members.Contains h.B.Body && protectedIds.Contains h.A.Body then Some h.A else None) |> List.distinct
              let inputs = edges |> List.filter (fun p -> p.Body<>ground.Id)
              let roots = inputs |> List.map _.Body |> List.distinct
              if roots.Length=1 && roots.Head<>s.Program.Motor.Body then
                for preferred,inputPort in [for preferred in [true;false] do for input in inputs -> preferred,input] do
                  let ceiling = if preferred && maxPerFoot>1 then maxPerFoot-1 else maxPerFoot
                  let inputBody = s.Program.Bodies |> List.find (fun b -> b.Id=inputPort.Body)
                  let inputBody = {inputBody with Initial=s.Frames.Head.Poses[inputBody.Id]}
                  let inputPoint = point s inputPort
                  let oldGround = edges |> List.filter (fun p -> p.Body=ground.Id) |> List.map (point s)
                  let oldKnee =
                      if foot.Port.Body.EndsWith("-sole-output") then
                          let oldPrefix = foot.Port.Body.Substring(0,foot.Port.Body.Length-"-sole-output".Length)
                          s.Program.Bodies |> List.tryFind (fun b ->
                              b.Id=oldPrefix+"-leg-a" && b.Sites |> List.exists (fun site -> site.Id="tip"))
                          |> Option.map (fun b -> point s (Motion.port b.Id "tip"))
                      else None
                  let oldPivots = g.Connections |> List.filter (fun c -> c.Hinges |> List.exists (fun id ->
                      let h = s.Program.Hinges |> List.find (fun h -> h.Id=id)
                      members.Contains h.A.Body || members.Contains h.B.Body))
                  let inputParts = (g.Bodies |> List.find (fun b -> b.Body=inputPort.Body)).Parts
                                   |> List.filter (fun p -> oldPivots |> List.exists (fun c -> WalkerStructural.engaged c.Placement p))
                  for support in groundParts |> List.filter (fun p -> (sockets p).Length>=2) |> List.collect supports do
                    let anchors = sockets support
                    if anchors |> List.exists (fun p -> oldGround |> List.exists (fun old -> near old.X p.X && near old.Y p.Y)) then
                      for lower in anchors |> List.sortBy (fun p ->
                          let nearest = oldGround |> List.map (fun old -> Vec3.distance (at 0.0 p) (at 0.0 old)) |> List.min
                          nearest,-p.Y) do
                        for upper in anchors |> List.sortBy (fun p -> abs(lower.Y-p.Y-80.0)) do
                          let distance = Vec3.distance (at 0.0 lower) inputBody.Initial.Origin
                          if near lower.X upper.X && [60.0;80.0;40.0] |> List.exists (fun height -> near (lower.Y-upper.Y) height)
                             && distance>20.000001 && distance<120.0 && lower.Y<inputBody.Initial.Origin.Y then
                            for inputPart in inputParts do
                              if abs(support.Position.Z-inputPart.Position.Z)<=40.0
                                 && (Catalog.get inputPart.Part).Ports |> List.exists (fun socket ->
                                  let p = Vec3.add inputPart.Position (Rotation.apply inputPart.Rotation socket.Position)
                                  near p.X inputPoint.X && near p.Y inputPoint.Y) then
                                let branches =
                                    match oldKnee with
                                    | Some knee when (Vec3.cross (Vec3.subtract inputPoint lower) (Vec3.subtract knee lower)).Z< -1e-6 -> [-1;1]
                                    | _ -> [1;-1]
                                for branch in branches do
                                  for toe in [40.0;-40.0] do
                                    if accepted<ceiling then
                                        let prefix = prefixFor s foot.Id
                                        let template = mechanism prefix inputBody inputPort.Site lower upper toe branch foot.Id
                                                       |> Result.bind (fun p -> Realization.screen p 72)
                                        match template with
                                        | Error error -> yield foot.Id,lower,toe,branch,0,Error error
                                        | Ok template when not preferred || outwardL template prefix ->
                                            match Realization.propose template 2 with
                                            | Error error -> yield foot.Id,lower,toe,branch,0,Error error
                                            | Ok choices ->
                                                for choice in choices do
                                                    if accepted<ceiling then
                                                        match Realization.compile template choice with
                                                        | Error error -> yield foot.Id,lower,toe,branch,choice.Side,Error error
                                                        | Ok fixture ->
                                                            let part = fixture.Assembly.Parts |> List.find (fun p -> p.Body=Some(prefix+"-upper-output"))
                                                            let placement : WalkerPlacement =
                                                                {Id=part.Id;Part=part.Part;Color=part.Color;Position=part.Position;Rotation=part.Rotation}
                                                            let preferredLayout = intendedL template prefix placement && (oldKnee.IsNone || branch=branches.Head)
                                                            if preferredLayout=preferred then
                                                                let bodies = g.Bodies |> List.map (fun body ->
                                                                    if body.Body=ground.Id then
                                                                        {body with Parts=body.Parts |> List.map (fun p -> if p.Id=support.Id then support else p)}
                                                                    else body)
                                                                let result =
                                                                    graft s {g with Bodies=bodies} foot members support inputPort inputPart prefix template fixture
                                                                    |> Result.map (fun edit ->
                                                                        let original = groundParts |> List.find (fun p -> p.Id=support.Id)
                                                                        let parameters = $"feet/{foot.Id}/parameters",[],[lower.X;lower.Y;upper.Y;toe;float branch;float choice.Side]
                                                                        let preference = $"feet/{foot.Id}/l-layout-preference",[],[if preferred then 1.0 else 0.0]
                                                                        let edit = {edit with Changes=edit.Changes @ [parameters;preference]}
                                                                        if original=support then edit else
                                                                            let changes = [
                                                                                $"parts/{support.Id}/upright:{original.Part}->{support.Part}",
                                                                                [original.Position.Y],[support.Position.Y]]
                                                                            {edit with Changes=edit.Changes @ changes})
                                                                if Result.isOk result then accepted <- accepted+1
                                                                yield foot.Id,lower,toe,branch,choice.Side,result
                                        | Ok _ -> ()
        ]
        |> List.distinct

    let attempts s g = attemptsWithLimit 32 s g

    let edits s g =
        attemptsWithLimit 4 s g |> List.choose (fun (_,_,_,_,_,result) -> match result with Ok edit -> Some edit | Error _ -> None)
        |> List.distinct
        |> List.mapi (fun rank edit ->
            {edit with Changes=edit.Changes @ ["construction/level-foot-rank",[],[float rank]]})
