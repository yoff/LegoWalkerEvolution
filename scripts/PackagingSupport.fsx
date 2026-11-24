#r "../src/LegoWalkers.Core/bin/Debug/net10.0/LegoWalkers.Core.dll"

open System
open LegoWalkers

type MassPart = {
    Id: string; Part: string; Body: string; MassKg: float
    Center: Vec3; Low: Vec3; High: Vec3; Source: string
}
type MassInput = {
    SchemaVersion: int; ParentManifestSha256: string; ProgramSha256: string
    RealizationSha256: string; DependencySha256: string; PayloadProfileSha256: string
    Parts: MassPart list
}
type FootSample = { Id: string; LogicalFoot: string; Position: Vec3; Radius: float }
type MotionSample = {
    InputAngleRadians: float; Feet: FootSample list; StanceFeet: string list
    SupportHull: Vec3 list; GroundY: float; Links: Vec3 list list
}
type BalanceSample = { Center: Vec3; MarginLdu: float; HeightLdu: float }
type Target = {
    Id: string; Part: string; MassKg: float; Position: Vec3; Rotation: Rotation
    Center: Vec3; Low: Vec3; High: Vec3; ToleranceLdu: Vec3
    InsideInheritedRailPlanes: bool; MovingClearance: WalkerClearanceReport
}
type Plan = {
    Id: string; Targets: Target list; Phases: BalanceSample list
    WorstMarginLdu: float; MeanHeightLdu: float; OutsideSupportPhases: int
    MovingClearanceScore: float; HousingOverlapPairs: string list list
    GroundIntrusionPhases: int
}

module PackagingMath =
    let cross (a: Vec3) (b: Vec3) (c: Vec3) = (b.X-a.X)*(c.Z-a.Z)-(b.Z-a.Z)*(c.X-a.X)
    let hull (points: Vec3 list) =
        let points = points |> List.distinct |> List.sortBy (fun p -> p.X,p.Z)
        if points.Length < 3 then points
        else
            let build sequence =
                let result =
                    sequence |> List.fold (fun side point ->
                        let rec trim = function
                            | b::a::rest when cross a b point <= 1e-9 -> trim (a::rest)
                            | side -> side
                        point::trim side) [] |> List.rev
                result |> List.take (result.Length-1)
            build points @ build (List.rev points)

    let margin (hull: Vec3 list) (center: Vec3) =
        let projection (p: Vec3) = Vec3.create p.X 0 p.Z
        let center = projection center
        match hull with
        | [] -> invalidArg "support" "Cannot rank a phase with no assumed supporting foot."
        | [a] -> -Vec3.distance (projection a) center
        | [a;b] ->
            let distance,_,_ = WalkerClearance.segmentDistance (projection a) (projection b) center center
            -distance
        | _ ->
            (hull @ [hull.Head]) |> List.pairwise
            |> List.map (fun (a,b) -> cross a b center / Vec3.distance (projection a) (projection b))
            |> List.min

    let stance band (feet: FootSample list) =
        if feet.IsEmpty || not (Double.IsFinite band) || band < 0.0 then
            invalidArg "stance" "Finite nonnegative stance band and at least one foot are required."
        let bottom = feet |> List.map (fun foot -> foot.Position.Y+foot.Radius) |> List.max
        let active = feet |> List.filter (fun foot -> bottom-foot.Position.Y-foot.Radius <= band+1e-8)
        let points = [
            for foot in active do
                for index in 0..11 do
                    let angle = float index*Math.PI/6.0
                    yield Vec3.create (foot.Position.X+foot.Radius*cos angle) 0
                                      (foot.Position.Z+foot.Radius*sin angle)
        ]
        bottom,active |> List.map _.Id,hull points

    let movePoint (initial: MotionPose) (current: MotionPose) point =
        let local = Rotation.apply (Rotation.aroundZ -initial.Angle) (Vec3.subtract point initial.Origin)
        Motion.transform current local

    let world (screened: ScreenedMotion) (frame: MotionFrame) body point =
        movePoint screened.Frames.Head.Poses[body] frame.Poses[body] point

    let bounds (position: Vec3) rotation (low: Vec3) (high: Vec3) =
        let points = [for x in [low.X;high.X] do
                          for y in [low.Y;high.Y] do
                              for z in [low.Z;high.Z] ->
                                  Vec3.add position (Rotation.apply rotation (Vec3.create x y z))]
        Vec3.create (points |> List.minBy _.X).X (points |> List.minBy _.Y).Y (points |> List.minBy _.Z).Z,
        Vec3.create (points |> List.maxBy _.X).X (points |> List.maxBy _.Y).Y (points |> List.maxBy _.Z).Z

    let boxGap (lo: Vec3,hi: Vec3) (otherLow: Vec3,otherHigh: Vec3) =
        let gap a b c d = max (c-b) (a-d)
        let x,y,z = gap lo.X hi.X otherLow.X otherHigh.X,gap lo.Y hi.Y otherLow.Y otherHigh.Y,
                    gap lo.Z hi.Z otherLow.Z otherHigh.Z
        if x <= 0.0 && y <= 0.0 && z <= 0.0 then max x (max y z)
        else sqrt (pown (max 0.0 x) 2+pown (max 0.0 y) 2+pown (max 0.0 z) 2)

    let overlap a b = boxGap a b < 0.0

    let meanOverAngles (angles: float array) (values: float array) =
        if angles.Length <> values.Length || angles.Length < 2
           || (values |> Array.exists (Double.IsFinite >> not))
           || (angles |> Array.exists (Double.IsFinite >> not)) then
            invalidArg "samples" "Finite matching angle/value arrays with at least two samples are required."
        let mutable integral = 0.0
        for i in 1..angles.Length-1 do
            let width = angles[i]-angles[i-1]
            if width < 0.0 then invalidArg "angles" "Reference angles must be nondecreasing."
            integral <- integral+width*(values[i-1]+values[i])/2.0
        let width = angles[angles.Length-1]-angles[0]
        if width <= 0.0 then invalidArg "angles" "A positive phase interval is required."
        integral/width

    let motionSamples (s: ScreenedMotion) (r: RealizedWalker) frames band =
        frames |> List.map (fun frame ->
            let feet = r.Assembly.Feet |> List.map (fun foot ->
                let logical = r.LogicalFeet[foot.Id]
                { Id=foot.Id;LogicalFoot=logical;Position=world s frame foot.Body foot.Position
                  Radius=(s.Program.Feet |> List.find (fun f -> f.Id=logical)).Radius })
            let ground,stance,hull = stance band feet
            let links = r.Genome.Bodies |> List.filter (fun b -> b.Body<>r.Assembly.Chassis)
                        |> List.collect (fun body -> body.Parts |> List.map (fun part ->
                            (Catalog.get part.Part).Ports |> List.map (fun port ->
                                Vec3.add part.Position (Rotation.apply part.Rotation port.Position) |> world s frame body.Body)))
            {InputAngleRadians=frame.InputAngle;Feet=feet;StanceFeet=stance;SupportHull=hull;GroundY=ground;Links=links})

    let hardwareClearance (s: ScreenedMotion) (frames: MotionFrame list) (g: WalkerGenome)
                          (coverage: WalkerClearanceReport) (hardware: WalkerPlacement) =
        let chassisId = s.Program.Bodies |> List.find _.Grounded |> _.Id
        let chassis = g.Bodies |> List.find (fun body -> body.Body=chassisId)
        let diagnostics = g.Bodies |> List.filter (fun body -> body.Body<>chassisId) |> List.map (fun body ->
            let diagnosticGenome = {g with Bodies=[{chassis with Parts=[hardware]};body];Connections=[];Mounts=[]}
            WalkerClearance.evaluateFrames s frames diagnosticGenome coverage.RefinedIntervals)
        for diagnostic in diagnostics do
            if diagnostic.SuspectPairs>diagnostic.Contacts.Length then
                invalidOp "Clearance contact truncation prevents complete packaging scoring."
        let contacts = diagnostics |> List.collect _.Contacts |> List.filter (fun c -> List.contains hardware.Id c.Parts)
                       |> List.sortBy (fun c -> c.ClearanceLdu,c.Parts)
        let status =
            if not coverage.EndpointRepeats || coverage.SamplingFailure.IsSome then "incomplete-sampling-warning"
            elif contacts.IsEmpty then "no-suspect-at-sampled-poses" else "warning"
        {coverage with Contacts=contacts |> List.truncate 32;SuspectPairs=contacts.Length
                       Score=contacts |> List.sumBy (fun c -> pown (WalkerClearance.warningMargin-c.ClearanceLdu) 2)
                       Status=status;UnmodeledParts=diagnostics |> List.collect _.UnmodeledParts |> List.distinct}
