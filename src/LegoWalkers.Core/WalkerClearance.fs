namespace LegoWalkers

open System
open System.IO

type WalkerClearanceContact = {
    Parts: string list
    Bodies: string list
    InputAngleRadians: float
    FirstInputAngleRadians: float
    ClearanceLdu: float
    PointA: Vec3
    PointB: Vec3
}

type WalkerClearanceReport = {
    Version: string
    Status: string
    Score: float
    RodRadiusLdu: float
    WarningMarginLdu: float
    SampledPoses: int
    RefinedIntervals: int
    MotorTurnsCovered: float
    EndpointRepeats: bool
    SamplingFailure: string option
    SuspectPairs: int
    Contacts: WalkerClearanceContact list
    UnmodeledParts: string list
    DynamicTrials: int
    Limitations: string list
}

module WalkerClearance =
    let version = "kinematic-clearance-v1"
    let rodRadius = 3.0
    let warningMargin = 2.0
    let private clamp low high value = max low (min high value)
    let private coordinates (v: Vec3) = [v.X;v.Y;v.Z]

    let segmentDistance a b c d =
        let u,v,w = Vec3.subtract b a,Vec3.subtract d c,Vec3.subtract a c
        let aa,bb,cc = Vec3.dot u u,Vec3.dot u v,Vec3.dot v v
        let dd,ee = Vec3.dot u w,Vec3.dot v w
        let s,t =
            if aa < 1e-12 && cc < 1e-12 then 0.0,0.0
            elif aa < 1e-12 then 0.0,clamp 0.0 1.0 (ee/cc)
            elif cc < 1e-12 then clamp 0.0 1.0 (-dd/aa),0.0
            else
                let determinant = aa*cc-bb*bb
                let s = if determinant > 1e-12 then clamp 0.0 1.0 ((bb*ee-cc*dd)/determinant) else 0.0
                let t = (bb*s+ee)/cc
                if t < 0.0 then clamp 0.0 1.0 (-dd/aa),0.0
                elif t > 1.0 then clamp 0.0 1.0 ((bb-dd)/aa),1.0
                else s,t
        let p,q = Vec3.add a (Vec3.scale s u),Vec3.add c (Vec3.scale t v)
        Vec3.distance p q,p,q

    let segmentBoxDistance a b low high =
        let start,delta = coordinates a,coordinates (Vec3.subtract b a)
        let axes = List.zip (List.zip start delta) (List.zip (coordinates low) (coordinates high))
        let breaks =
            [0.0;1.0] @ (axes |> List.collect (fun ((origin,d),(lo,hi)) ->
                if abs d < 1e-12 then []
                else [(lo-origin)/d;(hi-origin)/d] |> List.filter (fun t -> t > 0.0 && t < 1.0)))
            |> List.distinct |> List.sort
        let candidates = breaks @ [
            for left,right in List.pairwise breaks do
                let middle = (left+right)/2.0
                let active = axes |> List.choose (fun ((origin,d),(lo,hi)) ->
                    let point = origin+middle*d
                    if point < lo then Some (d,origin-lo)
                    elif point > hi then Some (d,origin-hi)
                    else None)
                let aa = active |> List.sumBy (fun (d,_) -> d*d)
                if aa > 1e-12 then
                    let bb = active |> List.sumBy (fun (d,offset) -> d*offset)
                    yield clamp left right (-bb/aa)
        ]
        candidates |> List.map (fun t ->
            let p = Vec3.add a (Vec3.scale t (Vec3.subtract b a))
            let q = Vec3.create (clamp low.X high.X p.X) (clamp low.Y high.Y p.Y) (clamp low.Z high.Z p.Z)
            Vec3.distance p q,p,q) |> List.minBy (fun (distance,_,_) -> distance)

    let private linkParts = Set.ofList ["32523.dat";"60483.dat";"32316.dat";"32524.dat";"32525.dat";
                                        "32526.dat";"32271.dat";"32063.dat";"6632.dat"]
    let internal bounds part =
        // Local housing envelopes measured from the pinned LDraw library, not exact collision surfaces.
        match part with
        | "58123.dat" -> Some (Vec3.create -40 -73.7777 -40,Vec3.create 40 24 40)
        | "58119.dat" -> Some (Vec3.create -110 -89 -40,Vec3.create 110 70 40)
        | "58121.dat" -> Some (Vec3.create -50 -49.36 0,Vec3.create 50 49.36 120)
        | "6632.dat" -> Some (Vec3.create -9 -5 -9,Vec3.create 9 5 49)
        | part -> Realization.bounds part

    let internal initialEnvelope (p: WalkerPlacement) =
            let local =
                match Catalog.gearTeeth p.Part with
                | Some teeth ->
                    let radius = float teeth*1.25+4.0
                    Some(Vec3.create -radius -radius -10,Vec3.create radius radius 10)
                | None when (Catalog.get p.Part).Connector.IsSome ->
                    let low,high = Catalog.shaftRange p.Part
                    Some(Vec3.create low -10 -10,Vec3.create high 10 10)
                | None -> bounds p.Part
            local |> Option.map (fun (lo,hi) ->
                let points = [for x in [lo.X;hi.X] do
                                  for y in [lo.Y;hi.Y] do
                                      for z in [lo.Z;hi.Z] ->
                                          Vec3.add p.Position (Rotation.apply p.Rotation (Vec3.create x y z))]
                Vec3.create ((points |> List.minBy _.X).X) ((points |> List.minBy _.Y).Y) ((points |> List.minBy _.Z).Z),
                Vec3.create ((points |> List.maxBy _.X).X) ((points |> List.maxBy _.Y).Y) ((points |> List.maxBy _.Z).Z))
    let internal envelopeOverlapRisk (a: (Vec3 * Vec3) option) (b: (Vec3 * Vec3) option) =
        match a,b with
        | Some(alo,ahi),Some(blo,bhi) ->
            max 0.0 (min (min ahi.X bhi.X-max alo.X blo.X)
                         (min (min ahi.Y bhi.Y-max alo.Y blo.Y) (min ahi.Z bhi.Z-max alo.Z blo.Z)))
        | _ -> 0.0

    let internal initialOverlapRisk a b =
        envelopeOverlapRisk (initialEnvelope a) (initialEnvelope b)

    let internal initialFootFloor (g: WalkerGenome) =
        let feet = g.Feet |> List.collect WalkerFootChoice.contacts |> List.map _.Part |> Set.ofList
        let envelopes =
            g.Bodies |> List.collect _.Parts |> List.filter (fun p -> feet.Contains p.Id)
            |> List.map (fun p -> p.Id,initialEnvelope p)
        if envelopes.Length<>feet.Count || envelopes |> List.exists (snd >> Option.isNone) then
            invalidArg "feet" "Initial foot clearance requires bounded envelopes for every physical foot."
        envelopes |> List.map (snd >> Option.get >> snd >> _.Y) |> List.max

    let internal initialGroundIntrusion floor (p: WalkerPlacement) =
        initialEnvelope p |> Option.map (fun (_,hi) -> max 0.0 (hi.Y-floor))
        |> Option.defaultWith (fun () -> invalidArg "part" $"No bounded initial envelope for {p.Id}.")

    let writeInitialGround directory (g: WalkerGenome) =
        let floor = initialFootFloor g
        let feet = g.Feet |> List.collect WalkerFootChoice.contacts |> List.map _.Part |> Set.ofList
        let parts = g.Bodies |> List.collect _.Parts
        let modeled,unmodeled = parts |> List.partition (fun p -> (initialEnvelope p).IsSome)
        let intrusions =
            modeled |> List.filter (fun p -> not (feet.Contains p.Id))
            |> List.map (fun p -> {| part = p.Id; intrusionLdu = initialGroundIntrusion floor p |})
            |> List.filter (fun p -> p.intrusionLdu>1e-6) |> List.sortByDescending _.intrusionLdu
        File.WriteAllText(Path.Combine(directory,"initial-ground-clearance.json"),Realization.json {|
            version = "initial-foot-clearance-v1"; authority = "advisory-only"
            footEnvelopeFloorYLdu = floor; intrusions = intrusions
            unmodeledParts = unmodeled |> List.map _.Id; dynamicTrials = 0
            limitations = ["Initial LDraw pose only; positive Y points down."
                           "Conservative housing/part envelopes, not exact permitted contact surfaces."
                           "No static-balance or whole-gait support requirement; exact reconstruction and simulation remain mandatory."]
        |}+"\n")

    let internal sweptEnvelope (initial: MotionFrame) (frames: MotionFrame list) body (part: WalkerPlacement) =
        let origin = initial.Poses[body]
        let offset = Rotation.apply (Rotation.aroundZ -origin.Angle) (Vec3.subtract part.Position origin.Origin)
        let boxes = frames |> List.choose (fun frame ->
            let pose = frame.Poses[body]
            let rotate = Rotation.apply (Rotation.aroundZ (pose.Angle-origin.Angle))
            initialEnvelope {part with Position=Motion.transform pose offset
                                       Rotation={X=rotate part.Rotation.X;Y=rotate part.Rotation.Y;Z=rotate part.Rotation.Z}})
        if boxes.IsEmpty then None
        else
            let lows,highs = List.unzip boxes
            Some(Vec3.create ((lows |> List.minBy _.X).X) ((lows |> List.minBy _.Y).Y) ((lows |> List.minBy _.Z).Z),
                 Vec3.create ((highs |> List.maxBy _.X).X) ((highs |> List.maxBy _.Y).Y) ((highs |> List.maxBy _.Z).Z))

    type private Shape = {
        Part: WalkerPlacement
        Body: string
        Rods: (Vec3 * Vec3) list
        Box: (Vec3 * Vec3) option
    }

    let private shapes (s: ScreenedMotion) (g: WalkerGenome) =
        let grounded = s.Program.Bodies |> List.filter _.Grounded |> List.map _.Id |> Set.ofList
        g.Bodies |> List.collect (fun body ->
            body.Parts |> List.map (fun part ->
                let rods =
                    if not (grounded.Contains body.Body) && linkParts.Contains part.Part then
                        let points = (Catalog.get part.Part).Ports |> List.map _.Position |> List.distinct
                        let reduced = points |> List.fold (fun acc point ->
                            match acc with
                            | last::previous::rest when
                                Vec3.length (Vec3.cross (Vec3.subtract last previous) (Vec3.subtract point last)) < 1e-8
                                && Vec3.dot (Vec3.subtract last previous) (Vec3.subtract point last) > 0.0 ->
                                point::previous::rest
                            | _ -> point::acc) [] |> List.rev
                        List.pairwise reduced
                    else []
                { Part = part; Body = body.Body; Rods = rods; Box = if rods.IsEmpty then bounds part.Part else None }))

    let private world (s: ScreenedMotion) (frame: MotionFrame) shape local =
        let initial = s.Frames.Head.Poses[shape.Body]
        let point = Vec3.add shape.Part.Position (Rotation.apply shape.Part.Rotation local)
        let relative = Rotation.apply (Rotation.aroundZ -initial.Angle) (Vec3.subtract point initial.Origin)
        Motion.transform frame.Poses[shape.Body] relative

    let private local (s: ScreenedMotion) (frame: MotionFrame) shape point =
        let initial,current = s.Frames.Head.Poses[shape.Body],frame.Poses[shape.Body]
        let relative = Rotation.apply (Rotation.aroundZ -current.Angle) (Vec3.subtract point current.Origin)
        let original = Vec3.subtract (Motion.transform initial relative) shape.Part.Position
        Vec3.create (Vec3.dot original shape.Part.Rotation.X) (Vec3.dot original shape.Part.Rotation.Y)
                    (Vec3.dot original shape.Part.Rotation.Z)

    let private envelope s frame shape =
        let radius = if shape.Rods.IsEmpty then 0.0 else rodRadius
        let points =
            match shape.Box with
            | Some (lo,hi) -> [for x in [lo.X;hi.X] do
                                  for y in [lo.Y;hi.Y] do
                                      for z in [lo.Z;hi.Z] -> world s frame shape (Vec3.create x y z)]
            | None -> shape.Rods |> List.collect (fun (a,b) -> [world s frame shape a;world s frame shape b])
        Vec3.create ((points |> List.minBy _.X).X-radius) ((points |> List.minBy _.Y).Y-radius) ((points |> List.minBy _.Z).Z-radius),
        Vec3.create ((points |> List.maxBy _.X).X+radius) ((points |> List.maxBy _.Y).Y+radius) ((points |> List.maxBy _.Z).Z+radius)

    let private jointRegions (s: ScreenedMotion) (g: WalkerGenome) frame a b =
        g.Connections |> List.collect (fun pivot ->
            if not (WalkerStructural.engaged pivot.Placement a.Part && WalkerStructural.engaged pivot.Placement b.Part) then []
            else
                s.Program.Hinges |> List.choose (fun hinge ->
                    if List.contains hinge.Id pivot.Hinges
                       && Set.ofList [hinge.A.Body;hinge.B.Body] = Set.ofList [a.Body;b.Body] then
                        let joint = Motion.position s.Graph frame hinge.A |> Result.defaultWith invalidOp
                        let lo,hi = Catalog.shaftRange pivot.Placement.Part
                        let depths = [pivot.Placement.Position.Z+lo*pivot.Placement.Rotation.X.Z
                                      pivot.Placement.Position.Z+hi*pivot.Placement.Rotation.X.Z]
                        Some (joint,List.min depths-rodRadius,List.max depths+rodRadius)
                    else None))

    let private trimJoints regions segments =
        regions |> List.fold (fun remaining ((joint: Vec3),low,high) ->
            remaining |> List.collect (fun ((a: Vec3),(b: Vec3)) ->
                let dx,dy = b.X-a.X,b.Y-a.Y
                let x,y = a.X-joint.X,a.Y-joint.Y
                let aa,bb,cc = dx*dx+dy*dy,2.0*(x*dx+y*dy),x*x+y*y-144.0
                let discriminant = bb*bb-4.0*aa*cc
                if min a.Z b.Z < low || max a.Z b.Z > high || discriminant < 0.0 then [a,b]
                elif aa < 1e-12 then if cc <= 0.0 then [] else [a,b]
                else
                    let left = max 0.0 ((-bb-sqrt discriminant)/(2.0*aa))
                    let right = min 1.0 ((-bb+sqrt discriminant)/(2.0*aa))
                    let at t = Vec3.add a (Vec3.scale t (Vec3.subtract b a))
                    if left >= right then [a,b]
                    else [if left > 1e-9 then yield a,at left
                          if right < 1.0-1e-9 then yield at right,b])) segments

    let private distances s g frame a b =
        let regions = jointRegions s g frame a b
        let rods shape =
            shape.Rods |> List.map (fun (x,y) -> world s frame shape x,world s frame shape y)
            |> trimJoints regions
        let rodsA,rodsB = rods a,rods b
        seq {
            for x,y in rodsA do
                for u,v in rodsB do
                    let distance,p,q = segmentDistance x y u v
                    yield distance-2.0*rodRadius,p,q
                match b.Box with
                | Some (lo,hi) ->
                    let distance,p,q = segmentBoxDistance (local s frame b x) (local s frame b y) lo hi
                    yield distance-rodRadius,world s frame b p,world s frame b q
                | None -> ()
            if a.Box.IsSome then
                let lo,hi = a.Box.Value
                for u,v in rodsB do
                    let distance,q,p = segmentBoxDistance (local s frame a u) (local s frame a v) lo hi
                    yield distance-rodRadius,world s frame a p,world s frame a q
        }

    let private scan s g modeled frame =
        let placed = modeled |> List.map (fun shape -> shape,envelope s frame shape) |> List.toArray
        [for i in 0..placed.Length-1 do
            let a,(al,ah) = placed[i]
            for j in i+1..placed.Length-1 do
                let b,(bl,bh) = placed[j]
                if a.Body <> b.Body && not (a.Rods.IsEmpty && b.Rods.IsEmpty)
                   && max al.X bl.X-min ah.X bh.X <= 10.0
                   && max al.Y bl.Y-min ah.Y bh.Y <= 10.0
                   && max al.Z bl.Z-min ah.Z bh.Z <= 10.0 then
                    let candidates = distances s g frame a b |> Seq.filter (fun (distance,_,_) ->
                        distance <= 10.0) |> Seq.toList
                    if not candidates.IsEmpty then
                        let distance,p,q = candidates |> List.minBy (fun (d,_,_) -> d)
                        yield { Parts = [a.Part.Id;b.Part.Id]; Bodies = [a.Body;b.Body]
                                InputAngleRadians = frame.InputAngle; FirstInputAngleRadians = frame.InputAngle
                                ClearanceLdu = distance; PointA = p; PointB = q }]

    let private repeats (s: ScreenedMotion) (frames: MotionFrame list) =
        let first,last = frames.Head,List.last frames
        s.Program.Bodies |> List.forall (fun body ->
            let a,b = first.Poses[body.Id],last.Poses[body.Id]
            Vec3.distance a.Origin b.Origin < 1e-5
            && abs (atan2 (sin (a.Angle-b.Angle)) (cos (a.Angle-b.Angle))) < 1e-6)

    let internal completeFrames (s: ScreenedMotion) =
        let steps = min 72 (s.Frames.Length-1)
        let mutable frames =
            if s.Frames.Length-1 <= 72 then s.Frames
            else
                let source = List.toArray s.Frames
                [for n in 0..steps -> source[int (Math.Round(float n*float (source.Length-1)/float steps))]]
        let mutable previous = List.last frames
        let mutable turns = 1
        let mutable failure = None
        while turns < 32 && failure.IsNone && not (repeats s frames) do
            let extra = [
                for index in 1..steps do
                    if failure.IsNone then
                        let angle = float (turns*steps+index)*2.0*Math.PI/float steps
                        match Motion.solve s.Graph angle (Some previous) with
                        | Ok next -> previous <- next; yield next
                        | Error error -> failure <- Some $"Sampling stopped at motor angle {angle:G8}: {error}"
            ]
            frames <- frames @ extra
            turns <- turns+1
        frames,failure

    let evaluateFrames s frames g refinedIntervals =
        let all = shapes s g
        let modeled = all |> List.filter (fun shape -> not shape.Rods.IsEmpty || shape.Box.IsSome)
        let minimums =
            frames |> List.collect (scan s g modeled)
            |> List.groupBy _.Parts |> List.map (fun (_,contacts) ->
                let worst = contacts |> List.minBy _.ClearanceLdu
                let suspect = contacts |> List.filter (fun c -> c.ClearanceLdu <= warningMargin)
                let first = if suspect.IsEmpty then worst.InputAngleRadians else (suspect |> List.minBy _.InputAngleRadians).InputAngleRadians
                { worst with FirstInputAngleRadians = first })
            |> List.filter (fun contact -> contact.ClearanceLdu <= warningMargin)
            |> List.sortBy (fun contact -> contact.ClearanceLdu,contact.Parts,contact.InputAngleRadians)
        { Version = version; Status = if minimums.IsEmpty then "no-suspect-at-sampled-poses" else "warning"
          Score = minimums |> List.sumBy (fun contact -> pown (warningMargin-contact.ClearanceLdu) 2)
          RodRadiusLdu = rodRadius; WarningMarginLdu = warningMargin; SampledPoses = frames.Length
          RefinedIntervals = refinedIntervals; MotorTurnsCovered = (List.last frames).InputAngle/(2.0*Math.PI)
          EndpointRepeats = repeats s frames; SuspectPairs = minimums.Length; Contacts = minimums |> List.truncate 32
          SamplingFailure = None
          UnmodeledParts = (all |> List.filter (fun shape -> shape.Rods.IsEmpty && shape.Box.IsNone) |> List.map _.Part.Id)
                           @ (g.Mounts |> List.map _.Id) @ (g.Connections |> List.map (fun c -> c.Placement.Id))
          DynamicTrials = 0
          Limitations = ["Advisory thin link cores and housing/structural boxes, not real collision meshes or a safety certificate."
                         "Local supported hinge contacts are excluded; other contacts between connected links remain checked."
                         "No box-box, connector, gear-tooth, ground, deformation or load-dependent collision checks."
                         "At most 72 primary intervals per turn and 32 motor turns; an unrepeated endpoint means incomplete cycle coverage."
                         "Up to eight near-contact intervals refined at three interior phases; between-sample collisions can be missed."
                         "Lower score only ranks approximate layouts. All existing physical and dynamics gates remain mandatory."] }

    let inspect s g =
        let frames,failure = completeFrames s
        let modeled = shapes s g |> List.filter (fun shape -> not shape.Rods.IsEmpty || shape.Box.IsSome)
        let near = frames |> List.map (fun frame ->
            scan s g modeled frame |> List.map _.ClearanceLdu |> function [] -> Double.PositiveInfinity | values -> List.min values)
        let intervals =
            List.zip (List.pairwise frames) (List.pairwise near)
            |> List.filter (fun (_, (a,b)) -> min a b <= 10.0)
            |> List.sortBy (fun ((a,_),(x,y)) -> min x y,a.InputAngle) |> List.truncate 8
        let mutable refinementFailure = None
        let extra = intervals |> List.collect (fun ((a,b),_) ->
            let mutable previous = a
            [for fraction in [0.25;0.5;0.75] do
                let angle = a.InputAngle+fraction*(b.InputAngle-a.InputAngle)
                match Motion.solve s.Graph angle (Some previous) with
                | Ok frame -> previous <- frame; yield frame
                | Error error -> refinementFailure <- Some $"Refinement unavailable at motor angle {angle:G8}: {error}"])
        let sampled = frames @ extra |> List.sortBy _.InputAngle
        let report = evaluateFrames s sampled g intervals.Length
        let samplingFailure = failure |> Option.orElse refinementFailure
        sampled,{ report with SamplingFailure = samplingFailure
                              Status = if samplingFailure.IsSome || not report.EndpointRepeats then "incomplete-sampling-warning" else report.Status }

    let write directory report =
        File.WriteAllText(Path.Combine(directory,"motion-clearance.json"),Realization.json report+"\n")

    let writePhases directory (s: ScreenedMotion) (r: RealizedWalker) frames (report: WalkerClearanceReport) =
        File.WriteAllText(Path.Combine(directory,"motion-phases.json"),Realization.json {|
            version = "prescribed-motion-phases-v1"; mechanismId = s.MechanismId
            genotypeId = r.GenotypeId; assemblyId = r.AssemblyId
            endpointRepeats = report.EndpointRepeats; samplingFailure = report.SamplingFailure
            parts = r.Assembly.Parts |> List.map _.Id |> List.sort
            frames = frames |> List.map (fun (frame: MotionFrame) -> {|
                inputAngleRadians = frame.InputAngle
                bodies = frame.Poses |> Map.toList |> List.map (fun (id,pose) -> {|
                    id = id; originLdu = [pose.Origin.X;pose.Origin.Y;pose.Origin.Z]; angleRadians = pose.Angle
                |})
            |})
            limitations = ["Prescribed closed-chain poses, not simulated motion or walking qualification."
                           "Finite phase samples; between-sample collisions and load-dependent deformation remain possible."]
        |}+"\n")

    let writeRealization directory s (r: RealizedWalker) =
        let destination = WalkerRealization.write directory s r
        let frames,report = inspect s r.Genome
        write destination report
        writePhases destination s r frames report
        writeInitialGround destination r.Genome
        destination
