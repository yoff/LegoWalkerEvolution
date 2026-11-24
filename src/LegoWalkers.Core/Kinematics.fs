namespace LegoWalkers

open System
open System.Collections.Concurrent

type LegPose = { A: Vec3; B: Vec3; C: Vec3; D: Vec3; CrankTip: Vec3; Foot: Vec3; RockerTip: Vec3 }

module Kinematics =
    let private validationCache = ConcurrentDictionary<int*int*int, string list>()
    let private pathCache = ConcurrentDictionary<int*int*int*int, FootPath>()

    let solve (g: Genome) angle =
        match MotionExamples.fourBar g angle with
        | Error error -> Error error
        | Ok program ->
            match Motion.compile program with
            | Error errors -> Error (String.concat "; " errors)
            | Ok graph ->
                match Motion.solve graph angle None with
                | Error error -> Error error
                | Ok frame ->
                    let point body site =
                        match Motion.position graph frame (Motion.port body site) with
                        | Ok value -> value
                        | Error error -> invalidOp error
                    Ok {
                        A = point "ground" "drive"
                        B = point "crank" "tip"
                        C = point "coupler" "joint"
                        D = point "ground" "rocker"
                        CrankTip = point "crank" "end"
                        Foot = point "coupler" "foot"
                        RockerTip = point "rocker" "end"
                    }

    let private pointSegment p a b =
        let ab = Vec3.subtract b a
        let t = Math.Clamp(Vec3.dot (Vec3.subtract p a) ab / Vec3.dot ab ab, 0.0, 1.0)
        Vec3.distance p (Vec3.add a (Vec3.scale t ab))

    let private segmentsTouch a b c d =
        let cross2 (u: Vec3) (v: Vec3) = u.X*v.Y-u.Y*v.X
        let ab, cd = Vec3.subtract b a, Vec3.subtract d c
        let denom = cross2 ab cd
        let intersects =
            if abs denom < 1e-9 then false
            else
                let delta = Vec3.subtract c a
                let t, u = cross2 delta cd / denom, cross2 delta ab / denom
                t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0
        intersects || List.min [pointSegment a c d; pointSegment b c d; pointSegment c a b; pointSegment d a b] < 18.0-1e-6

    let validate g =
        let errors = Genome.validate g
        if not errors.IsEmpty then errors
        else
            // Payload/category mutations do not change the eight possible leg geometries.
            validationCache.GetOrAdd((g.GroundHoles, g.CouplerHoles, g.RockerHoles), fun _ ->
                let poses = [for n in 0..719 -> solve g (float n * Math.PI/360.0)]
                let failures = poses |> List.choose (function Error e -> Some e | _ -> None) |> List.distinct
                if not failures.IsEmpty then failures
                else
                    let valid = poses |> List.choose (function Ok p -> Some p | _ -> None)
                    [
                        if valid |> List.exists (fun p -> segmentsTouch p.A p.CrankTip p.D p.RockerTip) then
                            "The coplanar crank and rocker envelopes interfere during a full turn."
                    ])

    let footPath g phase =
        let compute () =
            let points =
                [for n in 0..360 ->
                    match solve g (phase + float n * Math.PI/180.0) with
                    | Ok pose -> pose.Foot
                    | Error e -> invalidArg (nameof g) e]
            {
                Id = "coupler-tip"
                Points = points
                Stride = (points |> List.maxBy _.X).X - (points |> List.minBy _.X).X
                Lift = (points |> List.maxBy _.Y).Y - (points |> List.minBy _.Y).Y
            }
        let quarter = phase/(Math.PI/2.0)
        if Genome.validate g |> List.isEmpty
           && Double.IsFinite quarter && quarter >= -4.0 && quarter <= 8.0
           && abs (quarter - Math.Round quarter) < 1e-12 then
            pathCache.GetOrAdd((g.GroundHoles, g.CouplerHoles, g.RockerHoles, int (Math.Round quarter)), fun _ -> compute ())
        else compute ()
