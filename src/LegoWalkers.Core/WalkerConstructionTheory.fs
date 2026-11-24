namespace LegoWalkers

open System
open System.Collections.Generic

module WalkerConstructionTheory =
    let version = "walker-construction-theory-v22"

    type private Alternative<'a> = {
        Key: string
        Parts: WalkerPlacement list
        Bodies: string list
        Cost: float
        Value: 'a
        Introduces: ConstructionGoal list
        Bonds: ConstructionBond list
    }

    let private goal id kind subject requires external =
        {Id=id;Kind=kind;Subject=subject;Requires=requires;External=external}

    let private identity value = Export.sha256 (Realization.json value)

    let scope request oracleScope =
        if String.IsNullOrWhiteSpace oracleScope then invalidArg "oracleScope" "A frozen external oracle scope is required."
        identity (version,ConstructionSearch.version,WalkerPackaging.version,request,oracleScope)

    let private resolve (proof: ConstructionProof) (goal: ConstructionGoal) rule (alternatives: Alternative<'a> list) =
        let choices = alternatives |> List.map (fun alternative ->
            {Step={Goal=goal.Id;Rule=rule;Choice=alternative.Key;Dependencies=goal.Requires
                   Parts=alternative.Parts |> List.map _.Id;Bodies=alternative.Bodies;Cost=alternative.Cost
                   Introduced=alternative.Introduces |> List.map _.Id;Bonds=alternative.Bonds}
             Introduces=alternative.Introduces})
        match proof.Steps |> List.tryFind (fun s -> s.Goal=goal.Id) with
        | None -> Error(Branch(goal.Id,choices))
        | Some step ->
            let index = choices |> List.tryFindIndex (fun c -> c.Step=step)
            match index with
            | Some index ->
                for introduced in alternatives[index].Introduces do
                    if not(List.contains introduced proof.Goals) then
                        invalidArg "checkpoint" "An introduced obligation differs from its production."
                Ok alternatives[index].Value
            | None -> invalidArg "checkpoint" $"Production {goal.Id}/{step.Choice} cannot be replayed from these immutable inputs."

    let private detailGoals (request: WalkerPackagingRequest) =
        let ground = request.Program.Bodies |> List.find _.Grounded |> _.Id
        let driven = request.Drive.Gears |> List.map fst |> Set.ofList
        let supports = [
            for target in request.Targets do
                let sockets = Catalog.mountPorts target.Placement.Part
                for index,_ in List.indexed sockets do
                    yield $"packaging-{target.Placement.Id}-{index}",[ground]
            for hinge in request.Program.Hinges do
                let body = if hinge.A.Body=ground then hinge.B.Body else hinge.A.Body
                if (hinge.A.Body=ground || hinge.B.Body=ground) && driven.Contains body then
                    yield "axle-"+body,[body]
                    let gears = request.Drive.Gears |> List.filter (fun (b,_) -> b=body) |> List.map snd
                    let planes = WalkerPackaging.driveBearingPlanes request body gears
                    for index in 0..planes.Length-1 do
                        yield $"packaging-bearing-{body}-{index}",[body;ground]
        ]
        let mutable previousSupport = None
        let details = supports |> List.map (fun (id,bodies) ->
            let axle = id.StartsWith "axle-"
            let requirements =
                if axle then ["walking";"drive";"targets";"motor-mount"]
                else ["frame";"walking";"targets";"drive";"motor-mount"]
                     @ (if id.StartsWith "packaging-bearing-" then ["axle-"+bodies.Head] else [])
                     @ Option.toList previousSupport
            if not axle then previousSupport <- Some id
            goal id (if axle then "axle-stock" else "mount-and-bearing") (String.concat "," bodies) requirements false)
        let brackets = details |> List.filter (fun g -> g.Kind="mount-and-bearing")
        let bondGoals = brackets |> List.map (fun bracket ->
            goal ("bonds-"+bracket.Id) "rigid-port-bonds" bracket.Id ["frame";"targets";bracket.Id] false)
        let beamGoals = brackets |> List.collect (fun bracket ->
            let corners =
                if bracket.Id.StartsWith($"packaging-{request.Source.MotorPart}-") then ["-corner"] else []
            ["-anchor-beam";"-socket-beam"] @ corners |> List.map (fun suffix ->
                goal ("rigid-path-"+bracket.Id+suffix) "rigid-path-to-frame" (bracket.Id+suffix)
                     ["frame";"bonds-"+bracket.Id] false))
        let targetGoals = request.Targets |> List.map (fun target ->
            let mounts = brackets |> List.filter (fun bracket -> bracket.Id.StartsWith($"packaging-{target.Placement.Id}-"))
            goal ("rigid-path-"+target.Placement.Id) "rigid-path-to-frame" target.Placement.Id
                 ("frame"::(mounts |> List.map (fun g -> "bonds-"+g.Id))) false)
        let _,removed = WalkerPackaging.scaffold request
        let bearingIds = request.Program.Hinges |> List.filter (fun hinge ->
            (hinge.A.Body=ground && driven.Contains hinge.B.Body) ||
            (hinge.B.Body=ground && driven.Contains hinge.A.Body)) |> List.map _.Id |> Set.ofList
        let physical = ["frame";"walking";"targets";"drive"] @ (details |> List.map _.Id)
        let preservation =
            request.Source.Connections |> List.filter (fun pivot ->
                not(removed.Contains pivot.Placement.Id) && not(pivot.Hinges |> List.exists bearingIds.Contains))
            |> List.map (fun pivot ->
                goal ("preserve-"+pivot.Placement.Id) "exact-joint-preservation" pivot.Placement.Id physical true)
        goal "motor-mount" "motor-mount-motif" request.Source.MotorPart ["frame";"walking";"targets";"drive"] false
        :: details @ bondGoals @ beamGoals @ targetGoals
        @ preservation
        @ [goal "library-connectivity" "exact-rigid-connectivity" ground
                ((beamGoals @ targetGoals) |> List.map _.Id) true]

    let private reachable roots (bonds: ConstructionBond list) =
        let mutable connected = roots
        let mutable changed = true
        while changed do
            let next = bonds |> List.fold (fun known bond ->
                if Set.contains bond.PartA known || Set.contains bond.PartB known then
                    known |> Set.add bond.PartA |> Set.add bond.PartB
                else known) connected
            changed <- next<>connected
            connected <- next
        connected

    let private additionalBeamGoals prefix ids =
        ids |> List.filter (fun (id: string) -> id.StartsWith(prefix+"-bridge-beam-"))
        |> List.map (fun id -> goal ("rigid-path-"+id) "rigid-path-to-frame" id ["frame";"bonds-"+prefix] false)

    let bracketBonds prefix (parts: WalkerPlacement list) (frame: WalkerPlacement list) (targets: WalkerPlacement list) =
        let socketBeam =
            [5;4;3] |> List.tryFind (fun count -> parts |> List.exists (fun p -> p.Id=prefix + $"-bridge-beam-{count}"))
            |> Option.map (fun count -> prefix + $"-bridge-beam-{count}")
            |> Option.defaultValue (prefix+"-socket-beam")
        parts |> List.filter (fun pin -> (Catalog.get pin.Part).Connector=Some FrictionPin)
        |> List.collect (fun pin ->
            let endpoints =
                if pin.Id=prefix+"-anchor-pin" then
                    frame,parts |> List.filter (fun p -> p.Id=prefix+"-anchor-beam")
                elif pin.Id=prefix+"-elbow-pin" then
                    parts |> List.filter (fun p -> p.Id=prefix+"-anchor-beam"),
                    parts |> List.filter (fun p -> p.Id=prefix+"-socket-beam")
                elif pin.Id=prefix+"-mount-pin" then
                    parts |> List.filter (fun p -> p.Id=socketBeam),
                    (if parts |> List.exists (fun p -> p.Id=prefix+"-corner") then
                         parts |> List.filter (fun p -> p.Id=prefix+"-corner")
                     else targets |> List.filter (fun p -> prefix.StartsWith($"packaging-{p.Id}-")))
                elif pin.Id=prefix+"-side-pin" then
                    parts |> List.filter (fun p -> p.Id=prefix+"-corner"),
                    targets |> List.filter (fun p -> prefix.StartsWith($"packaging-{p.Id}-"))
                elif pin.Id.StartsWith(prefix+"-bridge-pin-") then
                    let count = [3..5] |> List.find (fun count -> pin.Id=prefix + $"-bridge-pin-{count}")
                    let previous = if count=3 then prefix+"-socket-beam" else prefix + $"-bridge-beam-{count-1}"
                    parts |> List.filter (fun p -> p.Id=previous),
                    parts |> List.filter (fun p -> p.Id=prefix + $"-bridge-beam-{count}")
                else invalidArg "bracket" $"Unknown rigid pin role {pin.Id}."
            let engaged structural = structural |> List.choose (fun part ->
                WalkerStructural.engagedPorts pin part |> List.tryHead
                |> Option.map (fun (port,point) -> part,point,Rotation.apply part.Rotation port.Axis))
            match engaged (fst endpoints),engaged (snd endpoints) with
            | (a,pointA,axisA)::_,(b,pointB,axisB)::_ ->
                [{Connector=pin.Id;PartA=a.Id;PositionA=pointA;AxisA=axisA
                  PartB=b.Id;PositionB=pointB;AxisB=axisB}]
            | _ -> [])

    let private goals (request: WalkerPackagingRequest) =
        let ground = request.Program.Bodies |> List.find _.Grounded |> _.Id
        [goal "frame" "retained-scaffold" ground [] false
         goal "walking" "link-and-connector-realization" "retained walking mechanism" ["frame"] false
         goal "targets" "component-placement" "battery,receiver,motor" ["frame";"walking"] false
         goal "drive" "required-drive-geometry" request.Drive.Root ["walking";"targets"] false]
        @ [goal "oracle" "exact-reconstruction-mass-collision-load" "complete assembly" [] true]

    let driveOptions (request: WalkerPackagingRequest) =
        let obsolete = Set.ofList request.ObsoletePartIds
        let outputPlanes = request.Drive.Gears |> List.choose (fun (body,gear) ->
            let parts = request.Source.Bodies |> List.tryFind (fun b -> b.Body=body) |> Option.map _.Parts |> Option.defaultValue []
            let sockets = parts |> List.filter (fun p -> not(obsolete.Contains p.Id)) |> List.collect (fun p ->
                (Catalog.get p.Part).Ports |> List.choose (fun port ->
                    let point = Vec3.add p.Position (Rotation.apply p.Rotation port.Position)
                    let axis = Rotation.apply p.Rotation port.Axis
                    if port.Kind=AxleHole && abs axis.Z>0.999
                       && abs(point.X-gear.Position.X)<1e-5 && abs(point.Y-gear.Position.Y)<1e-5 then Some point.Z else None))
            if body=request.Drive.Root || sockets.IsEmpty then None
            else Some(body,(List.average sockets)+40.0)) |> Map.ofList
        let rootPlane = request.Drive.Gears |> List.find (fun (body,_) -> body=request.Drive.Root) |> snd |> _.Position.Z
        let route fallbackPlane =
            let edges = request.Program.Transmissions |> List.map (fun edge ->
                let plane =
                    if edge.Input=request.Drive.Root || edge.Output=request.Drive.Root then rootPlane
                    else Map.tryFind edge.Input outputPlanes |> Option.orElseWith (fun () -> Map.tryFind edge.Output outputPlanes)
                         |> Option.defaultValue fallbackPlane
                edge,plane)
            let gears = request.Drive.Gears |> List.collect (fun (body,gear) ->
                let planes = edges |> List.choose (fun (edge,plane) ->
                    if edge.Input=body || edge.Output=body then Some plane else None) |> List.distinct |> List.sort
                planes |> List.mapi (fun index plane ->
                    let part =
                        if body<>request.Drive.Root && abs(abs request.Drive.MotorClockScale-5.0)<1e-9
                           && (request.Program.Transmissions |> List.exists (fun edge ->
                               edge.Input=request.Drive.Root && edge.Output=body))
                        then if abs(plane-rootPlane)<1e-6 then "3649.dat" else "3648b.dat"
                        else gear.Part
                    body,{gear with Id=(if index=0 then gear.Id else gear.Id+"-compound")
                                    Part=part;Position={gear.Position with Z=plane}}))
            {request with Drive={request.Drive with Gears=gears;InternalCompoundBearings=false}}
        let alternatives =
            rootPlane::(outputPlanes |> Map.toList |> List.map snd |> List.distinct |> List.sort)
            |> List.map route |> List.distinctBy (fun route -> identity route.Drive)
            |> List.filter (fun route -> route.Drive.Gears<>request.Drive.Gears && not route.Drive.Gears.IsEmpty)
        let interior = alternatives |> List.choose (fun route ->
            let groups = route.Drive.Gears |> List.groupBy fst |> List.map (fun (_,gears) -> gears |> List.map snd)
            let allowed =
                (groups |> List.exists (fun gears -> gears.Length=2)) &&
                (groups |> List.forall (fun gears ->
                    gears.Length<>2 || abs(gears[0].Position.Z-gears[1].Position.Z)>=WalkerPackaging.minimumBearingSpan gears[0] gears[1]-1e-6))
            if allowed then Some {route with Drive={route.Drive with InternalCompoundBearings=true}} else None)
        let retainedPins = request.Source.Connections |> List.map _.Placement |> List.filter (fun p -> not(obsolete.Contains p.Id))
        request::(interior @ alternatives)
        |> List.filter (fun route ->
            route.Program.Transmissions |> List.forall (fun edge ->
                [for bodyA,a in route.Drive.Gears do
                    for bodyB,b in route.Drive.Gears do
                        if bodyA=edge.Input && bodyB=edge.Output
                           && Catalog.matchesSpurMesh edge.Ratio a.Part a.Position b.Part b.Position then
                            yield a,b] |> List.length = 1))
        |> List.sortBy (fun route ->
            route.Drive.Gears |> List.sumBy (fun (_,gear) ->
                retainedPins |> List.sumBy (WalkerClearance.initialOverlapRisk gear)),
            not route.Drive.InternalCompoundBearings)
        |> fun routes ->
            if request.Drive.Gears |> List.countBy fst |> List.exists (fun (_,count) -> count>1) then [request]
            else routes

    let withMotorReduction reduction (request: WalkerPackagingRequest) =
        if reduction=1 then request
        elif reduction<>5 then invalidArg "reduction" "Only unit drive or the supported 8T-to-40T 5:1 root reduction is available."
        else
            let edge = request.Program.Transmissions |> List.filter (fun edge -> edge.Input=request.Drive.Root) |> List.exactlyOne
            let root = request.Drive.Gears |> List.find (fun (body,_) -> body=edge.Input) |> snd
            let output = request.Drive.Gears |> List.find (fun (body,_) -> body=edge.Output) |> snd
            let delta = Vec3.subtract root.Position output.Position
            if abs(sqrt(delta.X*delta.X+delta.Y*delta.Y)-60.0)>1e-6
               || Catalog.gearTeeth root.Part<>Some 24 || Catalog.gearTeeth output.Part<>Some 24
               || abs(abs edge.Ratio-1.0)>1e-9 || abs(abs request.Drive.MotorClockScale-1.0)>1e-9 then
                invalidArg "request" "5:1 reduction requires an unchanged 24T root mesh at 60 LDU spacing."
            let screened = Realization.screen request.Program 72 |> Result.defaultWith (Realization.json >> invalidArg "program")
            let ratio = edge.Ratio/5.0
            let phase = screened.Frames.Head.Poses[edge.Output].Angle-ratio*screened.Frames.Head.Poses[edge.Input].Angle
            let transmissions = request.Program.Transmissions |> List.map (fun t ->
                if t=edge then {t with Ratio=ratio;Phase=phase} else t)
            let program = {request.Program with Transmissions=transmissions}
            let plane = root.Position.Z+80.0
            let gears = request.Drive.Gears |> List.map (fun (body,gear) ->
                if body=edge.Input then body,{gear with Part="3647.dat";Position={gear.Position with Z=plane}}
                elif body=edge.Output then body,{gear with Part="3649.dat";Position={gear.Position with Z=plane}}
                else body,gear)
            {request with Program=program;Drive={request.Drive with Gears=gears;MotorClockScale=request.Drive.MotorClockScale*5.0}}

    let targetOptions (request: WalkerPackagingRequest) =
        let motor = request.Targets |> List.find (fun t -> t.Placement.Id=request.Source.MotorPart)
        let gears = (request.Drive.Gears |> List.map snd)
                    @ (driveOptions request |> List.collect (fun route -> route.Drive.Gears |> List.map snd))
                    |> List.distinct
        let shifted =
            [20.0;40.0;60.0;80.0] |> List.choose (fun setback ->
                let delta = Vec3.scale setback motor.Placement.Rotation.Z
                if abs delta.X<=motor.ToleranceLdu.X && abs delta.Y<=motor.ToleranceLdu.Y && abs delta.Z<=motor.ToleranceLdu.Z then
                    Some(request.Targets |> List.map (fun target ->
                        if target.Placement.Id=motor.Placement.Id then
                            {target with Placement={target.Placement with Position=Vec3.add target.Placement.Position delta}}
                        else target))
                else None)
        request.Targets::shifted
        |> List.sortBy (fun targets ->
            let placed = targets |> List.find (fun t -> t.Placement.Id=request.Source.MotorPart) |> _.Placement
            gears |> List.sumBy (WalkerClearance.initialOverlapRisk placed),
            // Reserve the side bracket's thickness plus 10 LDU clearance from rotating gear faces.
            gears |> List.sumBy (fun gear ->
                max 0.0 (30.0-abs(WalkerPackaging.motorSupportPlane placed-gear.Position.Z))),
            Vec3.distance placed.Position motor.Placement.Position)

    let start request oracleScope maximumStates maximumFrontier =
        WalkerPackaging.validate request |> Result.defaultWith (Realization.json >> invalidArg "request")
        ConstructionSearch.create (scope request oracleScope) (goals request) maximumStates maximumFrontier

    let motionTargetOptions (request: WalkerPackagingRequest) (screened: ScreenedMotion) =
        let walking = {request with Drive={request.Drive with Gears=[]}}
        WalkerPackaging.motionSpace walking screened |> Result.map (fun space ->
            let ground = screened.Program.Bodies |> List.find _.Grounded |> _.Id
            let frame,_ = WalkerPackaging.scaffold request
            let choices = request.Targets |> List.map (fun target ->
                let variants =
                    if target.Placement.Id=request.Source.MotorPart then
                        targetOptions request |> List.map (List.find (fun t -> t.Placement.Id=target.Placement.Id))
                    else
                        [Vec3.zero] @ [for distance in [-160.0..20.0..160.0] do
                                         yield Vec3.create distance 0 0
                                         yield Vec3.create 0 distance 0
                                         yield Vec3.create 0 0 distance]
                        |> List.filter (fun delta ->
                            abs delta.X<=target.ToleranceLdu.X && abs delta.Y<=target.ToleranceLdu.Y && abs delta.Z<=target.ToleranceLdu.Z)
                        |> List.map (fun delta -> {target with Placement={target.Placement with Position=Vec3.add target.Placement.Position delta}})
                        |> List.sortBy (fun t -> Vec3.distance t.Placement.Position target.Placement.Position,t.Placement.Position)
                variants |> List.distinct
                |> List.filter (fun t -> WalkerMotionSpace.fits space ground [t.Placement])
                |> List.filter (fun t -> WalkerMotionSpace.fitsFixed [t.Placement] frame)
                |> List.truncate (if target.Placement.Id=request.Source.MotorPart then 5 else 12)
                |> List.indexed)
            let combinations = choices |> List.fold (fun combinations variants ->
                [for cost,chosen in combinations do
                    for index,target in variants do yield cost+index,chosen @ [target]]) [0,[]]
            combinations |> List.filter (fun (_,targets) ->
                targets |> List.forall (fun target ->
                    WalkerMotionSpace.fitsFixed [target.Placement]
                        (targets |> List.filter (fun other -> other.Placement.Id<>target.Placement.Id) |> List.map _.Placement)))
            |> List.sortBy (fun (cost,targets) -> cost,Realization.json targets)
            |> List.truncate 32 |> List.map snd)

    let expansion (request: WalkerPackagingRequest) oracleScope =
        let expectedScope = scope request oracleScope
        let rootGoals = goals request
        let reference = Realization.screen request.Reference 72 |> Result.defaultWith (Realization.json >> invalidArg "reference")
        let screened = Realization.screen request.Program 72 |> Result.defaultWith (Realization.json >> invalidArg "program")
        let frames = Motion.sample screened.Graph (reference.Frames |> List.map (fun f -> f.InputAngle*request.Drive.MotorClockScale))
        let cachedMotion = reference,screened,frames
        let routeCache = Dictionary<string,(WalkerPlacement list * WalkerPlacement list) list>()
        let spaceCache = Dictionary<string,Result<WalkerMotionSpace.Space,GateDecision>>()
        let targetCache = Dictionary<string,Result<WalkerComponentTarget list list,GateDecision>>()
        let buildCache = Dictionary<string,Result<WalkerPackagingConstruction,GateDecision>>()
        let frame,remove = WalkerPackaging.scaffold request
        let ground = request.Program.Bodies |> List.find _.Grounded |> _.Id
        let knownBodies = request.Program.Bodies |> List.map _.Id |> Set.ofList
        let driveBodies = request.Drive.Gears |> List.map fst |> Set.ofList
        let bearingIds = request.Program.Hinges |> List.filter (fun h ->
            (h.A.Body=ground && driveBodies.Contains h.B.Body) || (h.B.Body=ground && driveBodies.Contains h.A.Body))
                         |> List.map _.Id |> Set.ofList
        let inherited (source: WalkerGenome) =
            (source.Bodies |> List.filter (fun b -> b.Body<>ground && knownBodies.Contains b.Body) |> List.collect _.Parts)
            @ (source.Connections |> List.filter (fun c -> not(c.Hinges |> List.exists bearingIds.Contains)) |> List.map _.Placement)
            |> List.filter (fun p -> not(remove.Contains p.Id))
        let fixedOption parts bodies value =
            [{Key=identity parts;Parts=parts;Bodies=bodies;Cost=0.0;            Value=value;Introduces=[];Bonds=[]}]
        let memo = Dictionary<string,ConstructionExpansion<WalkerPackagingConstruction>>()
        let repairsMemo = Dictionary<string,WalkerGenome list>()
        let expand (proof: ConstructionProof) (nogoods: ConstructionNogood list) =
            if proof.Scope<>expectedScope then invalidArg "checkpoint" "Construction request/oracle scope changed."
            let routes = driveOptions request
            let selectedDrive = proof.Steps |> List.tryFind (fun s -> s.Goal="drive")
            let selected = selectedDrive |> Option.map (fun step ->
                routes |> List.tryFind (fun route -> identity route.Drive=step.Choice)
                |> Option.defaultWith (fun () -> invalidArg "checkpoint" "Unknown drive-plane production."))
            let baseGoals = rootGoals @ (selected |> Option.map detailGoals |> Option.defaultValue [])
            let additional = proof.Steps |> List.collect (fun step ->
                if baseGoals |> List.exists (fun g -> g.Id=step.Goal && g.Kind="mount-and-bearing") then
                    additionalBeamGoals step.Goal step.Parts
                else [])
            let expectedGoals = baseGoals @ additional
            if proof.Goals<>expectedGoals then invalidArg "checkpoint" "Construction obligations differ from the frozen theory."
            ConstructionSearch.validate proof |> ignore
            let cacheKey = identity (ConstructionSearch.nodeId proof,nogoods)
            match memo.TryGetValue cacheKey with
            | true,result -> result
            | _ ->
                let byGoal = proof.Goals |> List.map (fun g -> g.Id,g) |> Map.ofList
                let frameParts = frame @ (request.Source.Mounts |> List.filter (fun p -> not(remove.Contains p.Id)))
                let result =
                    resolve proof byGoal["frame"] "retain-ground-scaffold" (fixedOption frameParts [ground] ())
                    |> Result.bind (fun () ->
                        let sourceParts =
                            ((request.Source.Bodies |> List.collect _.Parts) @ (request.Source.Connections |> List.map _.Placement))
                            |> List.map _.Id |> Set.ofList
                        let feedback = nogoods |> List.map _.Feedback |> List.filter (fun f ->
                            List.contains f.Reason ["rigid-body-partition";"joint-port-engagement";"initial-self-interference";"motion-clearance"])
                        let repaired = feedback |> List.collect (fun f ->
                            let parts = f.Parts |> List.filter sourceParts.Contains
                            let bodies = f.Bodies |> List.filter knownBodies.Contains
                            if (parts @ bodies).IsEmpty then []
                            else
                                let hint: WalkerConstructionFeedback = {Kind=f.Reason;Parts=parts;Bodies=bodies}
                                let key = identity hint
                                match repairsMemo.TryGetValue key with
                                | true,genomes -> genomes
                                | _ ->
                                    let choices = WalkerConstruction.repairs reference request.Source hint 8 |> List.map _.Genome
                                    repairsMemo.Add(key,choices)
                                    choices)
                        let options =
                            request.Source::repaired |> List.distinctBy identity |> List.map (fun source ->
                                {Key=identity source;Parts=inherited source;Bodies=source.Bodies |> List.filter (fun b -> b.Body<>ground) |> List.map _.Body
                                 Cost=(if source=request.Source then 0.0 else 5.0);Value=source;Introduces=[];Bonds=[]})
                        resolve proof byGoal["walking"] "import-or-feedback-link-and-pin-production" options)
                    |> Result.bind (fun source ->
                        let targetKey = identity source
                        let options =
                            match targetCache.TryGetValue targetKey with
                            | true,options -> options
                            | _ ->
                                let options = motionTargetOptions {request with Source=source} screened
                                targetCache.Add(targetKey,options)
                                options
                        let placements = options |> Result.map (List.mapi (fun index targets ->
                            {Key=identity (targets |> List.map _.Placement);Parts=targets |> List.map _.Placement;Bodies=[ground]
                             Cost=float index;Value=targets;Introduces=[];Bonds=[]}))
                        (match placements with
                         | Error gate -> Error(Blocked gate)
                         | Ok [] -> Error(Blocked {Gate="motion-space";Outcome="reject";Code="no-motion-clear-component-placement";
                                                  Diagnostics=["No bounded stock-envelope target assignment avoids the complete reserved gait."]})
                         | Ok placements -> resolve proof byGoal["targets"] "place-components-outside-reserved-gait" placements)
                        |> Result.bind (fun targets ->
                            let alternatives = routes |> List.mapi (fun index route ->
                                {Key=identity route.Drive;Parts=route.Drive.Gears |> List.map snd;Bodies=Set.toList driveBodies
                                 Cost=float index;Value=route;Introduces=detailGoals route;Bonds=[]})
                            resolve proof byGoal["drive"] "required-spur-drive-with-optional-compound-output-planes" alternatives
                            |> Result.map (fun route -> source,{route with Targets=targets})))
                    |> Result.bind (fun (source,route) ->
                        let mutable unresolved = None
                        let mutable replayed = Set.ofList ["frame";"walking";"targets";"drive"]
                        let choose key rule bodies (alternatives: WalkerPlacement list list) =
                            if unresolved.IsSome then invalidOp "The realizer continued beyond an unresolved production."
                            if not(byGoal.ContainsKey key) then invalidArg "checkpoint" $"Unknown construction obligation {key}."
                            let options = alternatives |> List.mapi (fun index parts ->
                                let cost = parts |> List.sumBy (fun p ->
                                    if (Catalog.get p.Part).Connector.IsSome then
                                        let low,high = Catalog.shaftRange p.Part
                                        (high-low)/100.0
                                    else 1.0)
                                let plannedMount = key="motor-mount"
                                {Key=identity parts;Parts=(if plannedMount then [] else parts);Bodies=bodies
                                 Cost=cost+float index;Value=index
                                 Introduces=(if plannedMount then [] else additionalBeamGoals key (parts |> List.map _.Id));Bonds=[]})
                            match resolve proof byGoal[key] rule options with
                            | Ok index -> replayed <- replayed.Add key;Some index
                            | Error branch -> unresolved <- Some branch;None
                        let physicalSteps = proof.Steps |> List.filter (fun step ->
                            not(List.contains byGoal[step.Goal].Kind ["rigid-port-bonds";"rigid-path-to-frame"]))
                        let physicalKey = identity (source,route.Drive,physicalSteps)
                        let outcome =
                            match buildCache.TryGetValue physicalKey with
                            | true,outcome ->
                                replayed <- physicalSteps |> List.map _.Goal |> Set.ofList
                                outcome
                            | _ ->
                                let outcome = WalkerPackaging.replay {route with Source=source} cachedMotion routeCache spaceCache choose
                                if unresolved.IsNone then buildCache.Add(physicalKey,outcome)
                                outcome
                        match unresolved,outcome with
                        | Some branch,_ -> Error branch
                        | None,Error gate -> Error(Blocked gate)
                        | None,Ok construction ->
                            if (physicalSteps |> List.map _.Goal |> Set.ofList)<>replayed then
                                invalidArg "checkpoint" "The proof contains steps not consumed by physical replay."
                            let produced = proof.Steps |> List.collect _.Parts |> Set.ofList
                            let physical = construction.Realized.Assembly.Parts |> List.map _.Id |> Set.ofList
                            if produced<>physical then invalidOp "Physical assembly differs from the proof's unique part witnesses."
                            Ok construction)
                    |> Result.bind (fun construction ->
                        let genome = construction.Realized.Genome
                        let placements =
                            (genome.Bodies |> List.collect _.Parts) @ genome.Mounts
                            @ (genome.Connections |> List.map _.Placement)
                            |> List.map (fun p -> p.Id,p) |> Map.ofList
                        let producers = proof.Steps |> List.collect (fun s -> s.Parts |> List.map (fun id -> id,s.Goal)) |> Map.ofList
                        let bondGoals = proof.Goals |> List.filter (fun g -> g.Kind="rigid-port-bonds")
                        let bondsResult = bondGoals |> List.fold (fun result bondGoal ->
                            result |> Result.bind (fun bonds ->
                                let step = proof.Steps |> List.find (fun s -> s.Goal=bondGoal.Subject)
                                let parts = step.Parts |> List.map (fun id -> placements[id])
                                let targets = request.Targets |> List.map (fun t -> placements[t.Placement.Id])
                                let next = bracketBonds bondGoal.Subject parts frame targets
                                let pins = parts |> List.filter (fun p -> (Catalog.get p.Part).Connector=Some FrictionPin)
                                let unsupported = pins |> List.filter (fun pin -> not(next |> List.exists (fun b -> b.Connector=pin.Id)))
                                if not unsupported.IsEmpty then
                                    Error(Blocked {Gate="construction-connectivity";Outcome="reject";Code="unproved-rigid-pin-bond"
                                                   Diagnostics=unsupported |> List.map _.Id})
                                else
                                    let dependencies = next |> List.collect (fun b -> [b.Connector;b.PartA;b.PartB])
                                                       |> List.map (fun id -> producers[id])
                                    let bondedGoal = {bondGoal with Requires=List.distinct(bondGoal.Requires @ dependencies)}
                                    let alternative =
                                        {Key=identity next;Parts=[];Bodies=[ground];Cost=0.0;Value=();Introduces=[];Bonds=next}
                                    resolve proof bondedGoal "coaxial-friction-pin-port-bonds" [alternative]
                                    |> Result.map (fun () -> bonds @ next))) (Ok [])
                        bondsResult |> Result.bind (fun bonds ->
                            let roots = frame |> List.map _.Id |> Set.ofList
                            let connected = reachable roots bonds
                            let pathGoals = proof.Goals |> List.filter (fun g -> g.Kind="rigid-path-to-frame")
                            pathGoals |> List.fold (fun result pathGoal ->
                                result |> Result.bind (fun () ->
                                    if not(connected.Contains pathGoal.Subject) then
                                        Error(Blocked {Gate="construction-connectivity";Outcome="reject";Code="unproved-rigid-path"
                                                       Diagnostics=[pathGoal.Subject]})
                                    else
                                        let pathGoal = {pathGoal with Requires=List.distinct(pathGoal.Requires @ (bondGoals |> List.map _.Id))}
                                        let alternative =
                                            {Key=identity (pathGoal.Subject,roots,bonds);Parts=[];Bodies=[ground];Cost=0.0
                                             Value=();Introduces=[];Bonds=[]}
                                        resolve proof pathGoal "compose-rigid-path-to-retained-frame" [alternative])) (Ok ())
                            |> Result.map (fun () -> construction)))
                    |> function
                        | Ok construction -> Built(construction.Realized.AssemblyId+":"+construction.Realized.GenotypeId,construction)
                        | Error expansion -> expansion
                memo.Add(cacheKey,result)
                result
        expand

    let advance request oracleScope state =
        if state.Scope<>scope request oracleScope then invalidArg "checkpoint" "Frozen request/oracle scope changed."
        ConstructionSearch.next (expansion request oracleScope) state
