module LegoWalkers.SpatialMotionTests

open System
open LegoWalkers
open LegoWalkers.Spatial
open Xunit

let private crank length : MotionProgram = {
    Bodies = [
        Motion.ground "ground" [Motion.site "bearing" 0 0]
        Motion.rigid "crank" 0 0 0 [Motion.site "root" 0 0; Motion.site "toe" length 0]
    ]
    Hinges = [Motion.couple "bearing" (Motion.port "ground" "bearing") (Motion.port "crank" "root")]
    Motor = { Body = "crank"; Phase = 0.0 }
    Transmissions = []
    Feet = [Motion.foot "toe" "crank" "toe" 4]
}

let private example () : SpatialProgram =
    let planar = MotionExamples.fourBar Genome.defaults 0.5 |> Result.defaultWith failwith
    let hip = planar.Bodies |> List.find (fun b -> b.Id = "rocker") |> _.Initial.Origin
    let root = {
        planar with
            Bodies = planar.Bodies @ [Motion.rigid "hip-shaft" hip.X hip.Y 0.5 [Motion.site "root" 0 0]]
            Hinges = planar.Hinges @ [
                Motion.couple "hip-bearing" (Motion.port "ground" "rocker") (Motion.port "hip-shaft" "root")]
            Transmissions = [Motion.transmit "crank" "hip-shaft" 1 0]
    }
    let instances = [
        for n in 0..3 ->
            let sign = if n%2 = 0 then 1.0 else -1.0
            { Id = $"leg-{n}"; Template = "legs"
              Carrier = SpatialMotion.body "root" "rocker"
              Mount = {
                  Origin = Vec3.create 0 0 (float (n+1)*20.0)
                  Rotation = { X = Vec3.create 1 0 0; Y = Vec3.create 0 0 sign; Z = Vec3.create 0 -sign 0 }
              }
              Input = { Source = SpatialMotion.body "root" "hip-shaft"; Ratio = -1; Phase = float (n/2)*Math.PI } }
    ]
    { Root = root; Placement = SpatialMotion.identity
      Templates = [{ Id = "legs"; Program = crank 40 }]
      Instances = instances
      Feet = instances |> List.map (fun i -> { Id = i.Id; Instance = i.Id; Foot = "toe" }) }

let private compiled p = SpatialMotion.compile p |> Result.defaultWith (String.concat "; " >> failwith)
let private cycle p =
    SpatialMotion.sample (compiled p) [for n in 0..180 -> 0.5 + float n*Math.PI/90.0]
    |> Result.defaultWith failwith

[<Fact>]
let ``shared lift template follows oscillating yaw with carrier relative power input`` () =
    let p = example ()
    let frames = cycle p
    for frame in frames do
        Assert.True(frame.MaxResidual < 1e-7)
        let parent = frame.LocalFrames["root"]
        for instance in p.Instances do
            let expected = -(parent.Poses["hip-shaft"].Angle-parent.Poses["rocker"].Angle) + instance.Input.Phase
            Assert.Equal(expected, frame.LocalFrames[instance.Id].Poses["crank"].Angle, 7)
        for pose in frame.Poses.Values do Assert.True(Rotation.valid pose.Rotation)
    for id in frames.Head.Feet.Keys do
        Assert.True(Vec3.distance frames.Head.Feet[id] (List.last frames).Feet[id] < 1e-6)
    let points = frames |> List.map (fun frame -> frame.Feet["leg-0"])
    let a, b, c = points[0], points[30], points[60]
    let normal = Vec3.normalize (Vec3.cross (Vec3.subtract b a) (Vec3.subtract c a))
    let nonplanarity = points |> List.map (fun p -> abs (Vec3.dot normal (Vec3.subtract p a))) |> List.max
    Assert.True(nonplanarity > 5.0, $"Expected a spatial arc, got {nonplanarity} LDU.")
    let yaw = frames |> List.map (fun f -> f.LocalFrames["root"].Poses["rocker"].Angle)
    Assert.InRange(List.max yaw-List.min yaw, 0.1, 2.0)

[<Fact>]
let ``changing one shared leg template preserves instance mounting and relative phase`` () =
    let p = example ()
    let changed = { p with Templates = [{ p.Templates.Head with Program = crank 60 }] }
    let before, after = cycle p, cycle changed
    for a, b in List.zip before after do
        for instance in p.Instances do
            let pivot = a.Poses[SpatialMotion.body instance.Id "ground"].Origin
            Assert.Equal(40.0, Vec3.distance pivot a.Feet[instance.Id], 6)
            Assert.Equal(60.0, Vec3.distance pivot b.Feet[instance.Id], 6)
            Assert.Equal(a.LocalFrames[instance.Id].Poses["crank"].Angle,
                         b.LocalFrames[instance.Id].Poses["crank"].Angle, 7)
        Assert.Equal(Math.PI,
                     b.LocalFrames["leg-2"].Poses["crank"].Angle-b.LocalFrames["leg-0"].Poses["crank"].Angle, 7)

[<Fact>]
let ``invalid hierarchy coupling frames and local graphs fail explicitly`` () =
    let p = example ()
    let first = p.Instances.Head
    let invalid = [
        { p with Templates = [{ p.Templates.Head with Program = { (crank 40) with Hinges = [] } }] }
        { p with Instances = first :: p.Instances }
        { p with Instances = [{ first with Carrier = SpatialMotion.body "leg-0" "crank"; Input = { first.Input with Source = SpatialMotion.body "leg-0" "missing" } }] }
        { p with Instances = [{ first with Mount = SpatialMotion.identity }] }
        { p with Instances = [{ first with Input = { first.Input with Ratio = Double.NaN } }] }
        { p with Instances = [{ first with Input = { first.Input with Source = SpatialMotion.body "root" "crank" } }] }
        { p with Feet = p.Feet @ [p.Feet.Head] }
    ]
    for candidate in invalid do Assert.True((SpatialMotion.compile candidate).IsError)
    let graph = compiled p
    Assert.True((SpatialMotion.solve graph Double.NaN None).IsError)
    let previous = SpatialMotion.solve graph 0.5 None |> Result.defaultWith failwith
    Assert.True((SpatialMotion.solve graph 0.6 (Some { previous with LocalFrames = Map.empty })).IsError)

[<Fact>]
let ``root only composition preserves the existing planar solver exactly`` () =
    let planar = MotionExamples.fourBar Genome.defaults 0.5 |> Result.defaultWith failwith
    let p = { Root = planar; Placement = SpatialMotion.identity; Templates = []; Instances = []
              Feet = [{ Id = "foot"; Instance = "root"; Foot = "coupler-tip" }] }
    let graph = compiled p
    let frame = SpatialMotion.solve graph 1.1 None |> Result.defaultWith failwith
    let legacy = Motion.compile planar |> Result.defaultWith (String.concat "; " >> failwith)
    let expected = Motion.solve legacy 1.1 None |> Result.defaultWith failwith
    Assert.Equal<MotionFrame>(expected, frame.LocalFrames["root"])
    let foot = (Motion.footPositions legacy expected)["coupler-tip"]
    Assert.True(Vec3.distance foot frame.Feet["foot"] < 1e-10)

[<Fact>]
let ``coaxiality uses actual grounded sockets rather than guessed initial body origins`` () =
    let p = example ()
    let shifted = {
        p.Root with
            Bodies = p.Root.Bodies |> List.map (fun b ->
                if b.Id <> "ground" then b
                else { b with Sites = b.Sites @ [Motion.site "other-bearing" 20 0] })
    }
    let shifted = {
        shifted with
            Hinges = shifted.Hinges |> List.map (fun h ->
                if h.Id <> "hip-bearing" then h else { h with A = Motion.port "ground" "other-bearing" })
    }
    match SpatialMotion.compile { p with Root = shifted } with
    | Ok _ -> failwith "Separated bearing axes were accepted."
    | Error errors -> Assert.Contains(errors, fun e -> e.Contains "coaxial ground bearings")

[<Fact>]
let ``nested moving frames are sorted by dependency and cyclic carriers are rejected`` () =
    let p = example ()
    let outer = { p.Instances.Head with Id = "outer"; Template = "yaw" }
    let inner = {
        p.Instances.Head with
            Id = "inner"
            Carrier = SpatialMotion.body "outer" "rocker"
            Input = { p.Instances.Head.Input with Source = SpatialMotion.body "outer" "hip-shaft" }
    }
    let nested = {
        p with
            Templates = p.Templates @ [{ Id = "yaw"; Program = p.Root }]
            Instances = [inner; outer]
            Feet = [{ Id = "toe"; Instance = "inner"; Foot = "toe" }]
    }
    let frame = SpatialMotion.solve (compiled nested) 0.6 None |> Result.defaultWith failwith
    Assert.Equal(3, frame.LocalFrames.Count)
    let outerFrame = frame.LocalFrames["outer"]
    let expected = -(outerFrame.Poses["hip-shaft"].Angle-outerFrame.Poses["rocker"].Angle)
    Assert.Equal(expected, frame.LocalFrames["inner"].Poses["crank"].Angle, 7)
    let cyclic = {
        outer with
            Carrier = SpatialMotion.body "outer" "rocker"
            Input = { outer.Input with Source = SpatialMotion.body "outer" "hip-shaft" }
    }
    match SpatialMotion.compile { nested with Instances = [inner; cyclic] } with
    | Ok _ -> failwith "Cyclic carrier hierarchy was accepted."
    | Error errors -> Assert.Contains(errors, fun e -> e.Contains "acyclic hierarchy")
