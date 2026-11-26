namespace LegoWalkers

open System

type MotionGroundClearance = {
    MinimumNonFootClearanceLdu: float
    MaximumRollRadians: float
    MaximumPitchRadians: float
    StanceToleranceLdu: float
}

type MotionGroundPhase = {
    InputAngleRadians: float
    GroundYLdu: float
    SupportCenter: Vec3
    NominalBodyHeightLdu: float
    StanceFeet: string list
    SwingClearanceLdu: (string * float) list
}

type WalkerGroundPlan = {
    MechanismId: string
    Requirements: MotionGroundClearance
    Phases: MotionGroundPhase list
}

module WalkerMotionSpace =
    let version = "sampled-motion-ground-reservation-v2"
    let toleranceLdu = 0.25
    let defaultGroundClearance = {
        MinimumNonFootClearanceLdu=8.0;MaximumRollRadians=0.0;MaximumPitchRadians=0.0;StanceToleranceLdu=1.0
    }

    type private Box = { Center: Vec3; Axes: Vec3 list; Half: float list; Low: Vec3; High: Vec3 }
    type private Obstacle = {
        Part: WalkerPlacement; Body: string; Poses: (float * Box list) list
        Low: Vec3; High: Vec3
    }
    type Space = private {
        JointBodies: Map<string,Set<string>>
        Obstacles: Obstacle list
        Samples: int
        Turns: float
        Grounded: Set<string>
        GroundPlan: WalkerGroundPlan option
    }
    type Conflict = { Part: string; Obstacle: string; Body: string; InputAngleRadians: float }
    type Envelope = { Part: string; Body: string; Low: Vec3; High: Vec3 }

    let envelopes space =
        space.Obstacles |> List.map (fun obstacle ->
            {Part=obstacle.Part.Id;Body=obstacle.Body;Low=obstacle.Low;High=obstacle.High})

    let private box (center: Vec3) (axes: Vec3 list) (half: float list) =
        let extent = List.zip axes half |> List.fold (fun total (axis,h) ->
            Vec3.add total (Vec3.create (abs axis.X*h) (abs axis.Y*h) (abs axis.Z*h))) Vec3.zero
        {Center=center;Axes=axes;Half=half;Low=Vec3.subtract center extent;High=Vec3.add center extent}

    let private stockBoxes (part: WalkerPlacement) =
        let at point = Vec3.add part.Position (Rotation.apply part.Rotation point)
        let bounds low high =
            [box (at (Vec3.scale 0.5 (Vec3.add low high)))
                 [part.Rotation.X;part.Rotation.Y;part.Rotation.Z]
                 [(high.X-low.X)/2.0;(high.Y-low.Y)/2.0;(high.Z-low.Z)/2.0]]
        let links = ["32523.dat";"60483.dat";"32316.dat";"32524.dat";"32525.dat";"32526.dat";"32271.dat";"32063.dat";"6632.dat"]
        if List.contains part.Part links then
            let thickness = Catalog.socketHalfThickness part.Part
            let points = (Catalog.get part.Part).Ports |> List.map _.Position |> List.distinct
            List.pairwise points |> List.map (fun (a,b) ->
                let delta = Vec3.subtract b a
                let length = Vec3.length delta
                let along = Vec3.scale (1.0/length) delta
                let across = Vec3.cross (Vec3.create 0 1 0) along
                box (at (Vec3.scale 0.5 (Vec3.add a b)))
                    [Rotation.apply part.Rotation along;part.Rotation.Y;Rotation.apply part.Rotation across]
                    [length/2.0+9.0;thickness;9.0])
        else
            match Catalog.gearTeeth part.Part,(Catalog.get part.Part).Connector with
            | Some teeth,_ ->
                let radius = float teeth*1.25+4.0
                [box part.Position [Vec3.create 1 0 0;Vec3.create 0 1 0;Vec3.create 0 0 1] [radius;radius;10.0]]
            | _,Some connector ->
                let low,high = Catalog.shaftRange part.Part
                let radius = if connector=Axle then 6.0 else 8.001
                if abs part.Rotation.X.Z>0.999 then
                    [box (at (Vec3.create ((low+high)/2.0) 0 0))
                         [Vec3.create 0 0 1;Vec3.create 1 0 0;Vec3.create 0 1 0] [(high-low)/2.0;radius;radius]]
                else bounds (Vec3.create low -radius -radius) (Vec3.create high radius radius)
            | _ ->
                let local =
                    if part.Part="3460.dat" then Some(Vec3.create -80 -4 -10,Vec3.create 80 8 10)
                    elif part.Part="3713.dat" then Some(Vec3.create -9 -9 -10,Vec3.create 9 9 10)
                    elif part.Part="4265c.dat" then Some(Vec3.create -9 -9 -5,Vec3.create 9 9 5)
                    else WalkerClearance.bounds part.Part
                match local with
                | Some(low,high) -> bounds low high
                | None -> invalidArg "part" $"No motion-reservation envelope for {part.Id} ({part.Part})."

    let partEnvelope (part: WalkerPlacement) =
        let boxes = stockBoxes part
        {Part=part.Id;Body="candidate"
         Low=boxes |> List.map _.Low |> List.reduce (fun a b -> Vec3.create (min a.X b.X) (min a.Y b.Y) (min a.Z b.Z))
         High=boxes |> List.map _.High |> List.reduce (fun a b -> Vec3.create (max a.X b.X) (max a.Y b.Y) (max a.Z b.Z))}

    let private boundsOverlap (lowA: Vec3) (highA: Vec3) (lowB: Vec3) (highB: Vec3) =
        min highA.X highB.X-max lowA.X lowB.X>toleranceLdu
        && min highA.Y highB.Y-max lowA.Y lowB.Y>toleranceLdu
        && min highA.Z highB.Z-max lowA.Z lowB.Z>toleranceLdu

    let private overlaps (a: Box) (b: Box) =
        boundsOverlap a.Low a.High b.Low b.High &&
            let delta = Vec3.subtract b.Center a.Center
            let axes = a.Axes @ b.Axes @ [for x in a.Axes do for y in b.Axes -> Vec3.cross x y]
            axes |> List.forall (fun axis ->
                let length = Vec3.length axis
                if length<1e-10 then true else
                let axis = Vec3.scale (1.0/length) axis
                let radius shape = List.zip shape.Axes shape.Half |> List.sumBy (fun (v,h) -> abs(Vec3.dot v axis)*h)
                radius a+radius b-abs(Vec3.dot delta axis)>toleranceLdu)

    let private obstacle body part (poses: (float * Box list) list) =
        let boxes = poses |> List.collect snd
        let low = boxes |> List.map _.Low |> List.reduce (fun a b -> Vec3.create (min a.X b.X) (min a.Y b.Y) (min a.Z b.Z))
        let high = boxes |> List.map _.High |> List.reduce (fun a b -> Vec3.create (max a.X b.X) (max a.Y b.Y) (max a.Z b.Z))
        {Part=part;Body=body;Poses=poses;Low=low;High=high}

    let reserve (s: ScreenedMotion) (g: WalkerGenome) =
        let frames,failure = WalkerClearance.completeFrames s
        let first,last = frames.Head,List.last frames
        let repeats = s.Program.Bodies |> List.forall (fun body ->
            let a,b = first.Poses[body.Id],last.Poses[body.Id]
            Vec3.distance a.Origin b.Origin<1e-5
            && abs(atan2 (sin(a.Angle-b.Angle)) (cos(a.Angle-b.Angle)))<1e-6)
        if failure.IsSome || not repeats then
            Error {Gate="motion-space";Outcome="reject";Code="incomplete-reserved-gait"
                   Diagnostics=[defaultArg failure "Body poses do not repeat within the 32-turn sampling bound."]}
        else
            let grounded = s.Program.Bodies |> List.filter _.Grounded |> List.map _.Id |> Set.ofList
            let moving = [
                for body in g.Bodies do
                    if not(grounded.Contains body.Body) then
                        for part in body.Parts do yield body.Body,part
                for pivot in g.Connections do
                    let hinges = s.Program.Hinges |> List.filter (fun h -> List.contains h.Id pivot.Hinges)
                    let owner =
                        if (Catalog.get pivot.Placement.Part).Connector=Some Axle then
                            hinges |> List.collect (fun h -> [h.A.Body;h.B.Body])
                            |> List.tryFind (grounded.Contains >> not)
                        else hinges |> List.tryHead |> Option.map (fun h -> h.B.Body)
                    match owner with
                    | Some body when not(grounded.Contains body) -> yield body,pivot.Placement
                    | _ -> ()
            ]
            let obstacles = moving |> List.map (fun (body,part) ->
                let initial = s.Frames.Head.Poses[body]
                let local = Rotation.apply (Rotation.aroundZ -initial.Angle) (Vec3.subtract part.Position initial.Origin)
                let poses = frames |> List.map (fun frame ->
                    let pose = frame.Poses[body]
                    let rotate = Rotation.apply (Rotation.aroundZ (pose.Angle-initial.Angle))
                    let placed = {part with Position=Motion.transform pose local
                                            Rotation={X=rotate part.Rotation.X;Y=rotate part.Rotation.Y;Z=rotate part.Rotation.Z}}
                    frame.InputAngle,stockBoxes placed)
                obstacle body part poses)
            let jointBodies =
                s.Program.Hinges |> List.collect (fun h ->
                    let ids = ("pivot-"+h.Id) :: (g.Connections |> List.filter (fun c -> List.contains h.Id c.Hinges) |> List.map _.Placement.Id)
                    ids |> List.map (fun id -> id,Set.ofList [h.A.Body;h.B.Body]))
                |> List.groupBy fst
                |> List.map (fun (id,members) -> id,members |> List.map snd |> Set.unionMany)
                |> Map.ofList
            Ok {JointBodies=jointBodies;Obstacles=obstacles;Samples=frames.Length;Turns=last.InputAngle/(2.0*Math.PI)
                Grounded=grounded;GroundPlan=None}

    let private placed (s: ScreenedMotion) (frame: MotionFrame) body (part: WalkerPlacement) =
        let initial,pose = s.Frames.Head.Poses[body],frame.Poses[body]
        let rotate = Rotation.apply (Rotation.aroundZ (pose.Angle-initial.Angle))
        let local = Rotation.apply (Rotation.aroundZ -initial.Angle) (Vec3.subtract part.Position initial.Origin)
        {part with Position=Motion.transform pose local
                   Rotation={X=rotate part.Rotation.X;Y=rotate part.Rotation.Y;Z=rotate part.Rotation.Z}}

    let private corners shape = [
        for x in [-1.0;1.0] do
            for y in [-1.0;1.0] do
                for z in [-1.0;1.0] do
                    yield List.zip3 shape.Axes shape.Half [x;y;z]
                          |> List.fold (fun point (axis,half,sign) -> Vec3.add point (Vec3.scale (half*sign) axis)) shape.Center
    ]

    let groundPlan (requirements: MotionGroundClearance) (s: ScreenedMotion) (g: WalkerGenome) =
        let valid value = Double.IsFinite value && value>=0.0
        if not(List.forall valid [requirements.MinimumNonFootClearanceLdu;requirements.StanceToleranceLdu;
                                 requirements.MaximumRollRadians;requirements.MaximumPitchRadians])
           || requirements.MaximumRollRadians>=Math.PI/2.0 || requirements.MaximumPitchRadians>=Math.PI/2.0 then
            invalidArg "ground-clearance" "Finite nonnegative clearance and stance tolerance, and attitude bounds below 90 degrees, are required."
        let frames,failure = WalkerClearance.completeFrames s
        if failure.IsSome then invalidArg "ground-clearance" failure.Value
        let parts = g.Bodies |> List.collect (fun body -> body.Parts |> List.map (fun part -> part.Id,(body.Body,part))) |> Map.ofList
        let ground = s.Program.Bodies |> List.find _.Grounded |> _.Id
        let phases = frames |> List.map (fun frame ->
            let soles = g.Feet |> List.map (fun foot ->
                let points = WalkerFootChoice.contacts foot |> List.collect (fun contact ->
                    let body,part = parts[contact.Part]
                    let position = placed s frame body part
                    match Catalog.soleUnderside part.Part with
                    | Some height ->
                        [for x in [-40.0;40.0] do
                            for z in [-10.0;10.0] ->
                                Vec3.add position.Position (Rotation.apply position.Rotation (Vec3.create x height z))]
                    | None ->
                        let output = s.Program.Feet |> List.find (fun f -> f.Id=foot.Foot)
                        let point = Motion.position s.Graph frame output.Port |> Result.defaultWith invalidOp
                        [{point with Y=point.Y+output.Radius;Z=foot.Depth}])
                foot.Foot,points)
            let groundY = soles |> List.collect snd |> List.map _.Y |> List.max
            let bottom = soles |> List.collect snd |> List.filter (fun point -> point.Y>=groundY-1e-6)
            let center = Vec3.scale (1.0/float bottom.Length) (bottom |> List.fold Vec3.add Vec3.zero)
            let heights = soles |> List.map (fun (id,points) -> id,groundY-(points |> List.map _.Y |> List.max))
            {InputAngleRadians=frame.InputAngle;GroundYLdu=groundY;SupportCenter=center
             NominalBodyHeightLdu=groundY-frame.Poses[ground].Origin.Y
             StanceFeet=heights |> List.choose (fun (id,lift) -> if lift<=requirements.StanceToleranceLdu then Some id else None)
             SwingClearanceLdu=heights})
        {MechanismId=s.MechanismId;Requirements=requirements;Phases=phases}

    let reserveWithGround requirements s g =
        reserve s g |> Result.map (fun space ->
            {space with GroundPlan=requirements |> Option.map (fun requirements -> groundPlan requirements s g)})

    let private violatesGround (plan: WalkerGroundPlan) (phase: MotionGroundPhase) parts =
        seq {
            let r,p = plan.Requirements.MaximumRollRadians,plan.Requirements.MaximumPitchRadians
            for part in parts do
                let low = stockBoxes part |> List.collect corners |> List.map (fun point ->
                    let delta = Vec3.subtract point phase.SupportCenter
                    -delta.Y*cos r*cos p-abs delta.X*sin p-abs delta.Z*sin r) |> List.min
                if low<plan.Requirements.MinimumNonFootClearanceLdu-1e-6 then
                    yield {Part=part.Id;Obstacle="planned-ground-clearance";Body="ground";InputAngleRadians=phase.InputAngleRadians}
        }

    let auditGround (plan: WalkerGroundPlan) (s: ScreenedMotion) (g: WalkerGenome) =
        if plan.MechanismId<>s.MechanismId then invalidArg "ground-plan" "The ground plan belongs to a different mechanism."
        let frames,_ = WalkerClearance.completeFrames s
        if frames.Length<>plan.Phases.Length
           || List.exists2 (fun (frame: MotionFrame) (phase: MotionGroundPhase) ->
                frame.InputAngle<>phase.InputAngleRadians) frames plan.Phases then
            invalidArg "ground-plan" "The ground plan must cover the exact complete prescribed gait."
        if plan<>groundPlan plan.Requirements s g then
            invalidArg "ground-plan" "The ground plan must match the current physical soles and finite nominal stance geometry."
        let permitted = g.Feet |> List.collect WalkerFootChoice.contacts |> List.map _.Part |> Set.ofList
        List.zip frames plan.Phases |> List.collect (fun (frame,phase) ->
            let physical =
                (g.Bodies |> List.collect (fun body ->
                    body.Parts |> List.filter (fun part -> not(permitted.Contains part.Id))
                    |> List.map (placed s frame body.Body)))
                @ g.Mounts
                @ (g.Connections |> List.map (fun pivot ->
                    let hinge = s.Program.Hinges |> List.find (fun h -> List.contains h.Id pivot.Hinges)
                    let owner = if (Catalog.get pivot.Placement.Part).Connector=Some Axle then hinge.B.Body else hinge.A.Body
                    placed s frame owner pivot.Placement))
            violatesGround plan phase physical |> Seq.toList)

    let private intendedJoint (space: Space) owner (part: WalkerPlacement) (obstacle: Obstacle) =
        let declared (connector: WalkerPlacement) (memberPart: WalkerPlacement) body =
            (Catalog.get connector.Part).Connector.IsSome
            && (space.JointBodies |> Map.tryFind connector.Id |> Option.exists (Set.contains body))
            && WalkerStructural.engaged connector memberPart
        declared part obstacle.Part obstacle.Body || declared obstacle.Part part owner

    let private intersections space owner parts =
        seq {
            if space.Grounded.Contains owner then
                match space.GroundPlan with
                | Some plan ->
                    for phase in plan.Phases do yield! violatesGround plan phase parts
                | None -> ()
            for part in parts do
                let boxes = stockBoxes part
                for obstacle in space.Obstacles do
                    if part.Id<>obstacle.Part.Id && owner<>obstacle.Body
                       && (boxes |> List.exists (fun b -> boundsOverlap b.Low b.High obstacle.Low obstacle.High))
                       && not(intendedJoint space owner part obstacle) then
                        match obstacle.Poses |> List.tryFind (fun (_,placed) ->
                            boxes |> List.exists (fun a -> placed |> List.exists (overlaps a))) with
                        | Some(angle,_) -> yield {Part=part.Id;Obstacle=obstacle.Part.Id;Body=obstacle.Body;InputAngleRadians=angle}
                        | None -> ()
        }

    let conflicts space owner parts = intersections space owner parts |> Seq.toList
    let firstConflicts limit space owner parts = intersections space owner parts |> Seq.truncate limit |> Seq.toList
    let fits space owner parts = intersections space owner parts |> Seq.isEmpty

    let private fixedIntersections parts hardware =
        seq {
            for part in parts do
                let boxes = stockBoxes part
                for other in hardware do
                    let engagement =
                        ((Catalog.get part.Part).Connector.IsSome && WalkerStructural.engaged part other)
                        || ((Catalog.get other.Part).Connector.IsSome && WalkerStructural.engaged other part)
                    if part.Id<>other.Id && not engagement
                       && (stockBoxes other |> List.exists (fun b -> boxes |> List.exists (fun a -> overlaps a b))) then
                        yield {Part=part.Id;Obstacle=other.Id;Body="fixed-hardware";InputAngleRadians=0.0}
        }

    let fixedConflicts parts hardware = fixedIntersections parts hardware |> Seq.toList
    let fitsFixed parts hardware = fixedIntersections parts hardware |> Seq.isEmpty

    let addBearingAxis space body part =
        {space with Obstacles=space.Obstacles @ [obstacle body part [0.0,stockBoxes part]]}

    let describe space = {|
        version=version;sampledPoses=space.Samples;motorTurnsCovered=space.Turns
        groundPlan=space.GroundPlan
        movingParts=space.Obstacles.Length;penetrationToleranceLdu=toleranceLdu;dynamicTrials=0
        authority="Conservative stock-envelope placement constraint, not exact mesh or continuous clearance certification."
        limitations=["Finite prescribed samples; exact complete-assembly phase screening remains mandatory."
                     "Declared connector/socket engagement is allowed; foreign links are not exempted."
                     "Ground clearance uses nominal stance-derived height and explicit conservative attitude bounds, not a predicted dynamic body pose."
                     "Swing clearance is sampled reach, not verified obstacle negotiation."
                     "No ground-support, settling, deformation or walking qualification."]
    |}
