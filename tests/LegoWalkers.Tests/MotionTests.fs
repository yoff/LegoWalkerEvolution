module LegoWalkers.MotionTests

open System
open System.IO
open System.Text.Json
open System.Globalization
open System.Text.RegularExpressions
open LegoWalkers
open Xunit

let compiled program =
    match Motion.compile program with
    | Ok graph -> graph
    | Error errors -> failwith (String.concat "; " errors)

let solved graph angle previous =
    match Motion.solve graph angle previous with
    | Ok frame -> frame
    | Error error -> failwith error

let position graph frame body site =
    match Motion.position graph frame (Motion.port body site) with
    | Ok value -> value
    | Error error -> failwith error

let cycle graph =
    match Motion.sample graph [for n in 0..360 -> float n*Math.PI/180.0] with
    | Ok frames -> frames
    | Error error -> failwith error

[<Theory>]
[<InlineData(40.0, 40.0)>]
[<InlineData(60.0, 40.0)>]
[<InlineData(60.0, -40.0)>]
[<InlineData(80.0, -40.0)>]
let ``reusable parallelograms preserve orientation and one foot in both L directions`` height toe =
    let p = MotionExamples.doubleParallelogram height toe |> Result.defaultWith failwith
    Assert.Equal(8, p.Bodies.Length)
    Assert.Equal(10, p.Hinges.Length)
    Assert.Single(p.Feet) |> ignore
    Assert.Equal("shoe", p.Feet.Head.Port.Body)
    let graph = compiled p
    let frames = cycle graph
    for frame in frames do
        Assert.True(frame.MaxResidual < 1e-6)
        Assert.True(abs frame.Poses["upright-l"].Angle < 1e-6)
        Assert.True(abs frame.Poses["shoe"].Angle < 1e-6)
        let corner = position graph frame "upright-l" "corner"
        let branch = position graph frame "upright-l" "toe-link"
        Assert.True(abs (branch.X-corner.X-toe) < 1e-6)
    let firstFoot = (Motion.footPositions graph frames.Head)["foot"]
    let lastFoot = (Motion.footPositions graph (List.last frames))["foot"]
    Assert.True(Vec3.distance firstFoot lastFoot < 1e-6)

[<Fact>]
let ``parallelogram follows a moving reference rather than prescribing world orientation`` () =
    let p = MotionExamples.fourBar Genome.defaults 0.5 |> Result.defaultWith failwith
    let reference = Motion.port "coupler" "joint", Motion.port "coupler" "root"
    let driver = Motion.port "rocker" "joint", Motion.port "rocker" "root"
    let child = Motion.addParallelogram "copy" reference driver ("root","end") [Motion.site "extra" 0 20] p
                |> Result.defaultWith failwith
    Assert.Equal(p.Bodies.Length+2, child.Bodies.Length)
    Assert.Equal(p.Hinges.Length+3, child.Hinges.Length)
    for frame in cycle (compiled child) do
        let difference = frame.Poses["copy-output"].Angle-frame.Poses["coupler"].Angle
        Assert.True(abs (atan2 (sin difference) (cos difference)) < 1e-6)
    Assert.Equal<MotionFoot list>(p.Feet, child.Feet)
    Assert.Equal<MotionMotor>(p.Motor, child.Motor)

[<Fact>]
let ``parallelogram and hinged foot reject invalid roots names singularities and outputs`` () =
    let p = MotionExamples.fourBar Genome.defaults 0 |> Result.defaultWith failwith
    let reference = Motion.port "ground" "drive", Motion.port "ground" "rocker"
    let driver = Motion.port "crank" "root", Motion.port "crank" "tip"
    Assert.True((Motion.addParallelogram "flat" reference driver ("root","end") [] p).IsError)
    let p = MotionExamples.fourBar Genome.defaults 0.5 |> Result.defaultWith failwith
    let goodReference = Motion.port "coupler" "joint", Motion.port "coupler" "root"
    let goodDriver = Motion.port "rocker" "joint", Motion.port "rocker" "root"
    let add prefix outputs sites = Motion.addParallelogram prefix goodReference goodDriver outputs sites p
    Assert.True((add "" ("root","end") []).IsError)
    Assert.True((add "bad" ("root","root") []).IsError)
    Assert.True((add "bad" ("root","end") [Motion.site "root" 0 20]).IsError)
    Assert.True((Motion.addParallelogram "bad"
                    (Motion.port "coupler" "root", Motion.port "coupler" "joint") goodDriver ("root","end") [] p).IsError)
    Assert.True((Motion.addParallelogram "bad"
                    (Motion.port "missing" "root", Motion.port "missing" "tip") goodDriver ("root","end") [] p).IsError)
    Assert.True((Motion.addHingedFoot "shoe" goodReference goodDriver ("root","end")
                    (Motion.site "contact" 20 14) "missing" p).IsError)

[<Theory>]
[<InlineData("four-bar", 3, 4)>]
[<InlineData("walker1", 7, 10)>]
[<InlineData("jansen", 7, 10)>]
let ``known legs use the same primitives and close over a continuous full turn`` name moving hinges =
    let program = MotionExamples.named name |> Result.defaultWith failwith
    Assert.Equal(moving, program.Bodies |> List.filter (fun b -> not b.Grounded) |> List.length)
    Assert.Equal(hinges, program.Hinges.Length)
    let graph = compiled program
    let frames = cycle graph
    Assert.Equal(361, frames.Length)
    for frame in frames do
        Assert.True(frame.MaxResidual <= 1e-7)
        for hinge in program.Hinges do
            let a = Motion.position graph frame hinge.A |> Result.defaultWith failwith
            let b = Motion.position graph frame hinge.B |> Result.defaultWith failwith
            Assert.True(Vec3.distance a b < 1e-6, $"{name}: {hinge.Id}")
    for body in program.Bodies do
        Assert.True(Vec3.distance frames.Head.Poses[body.Id].Origin (List.last frames).Poses[body.Id].Origin < 1e-5)
        let difference = frames.Head.Poses[body.Id].Angle - (List.last frames).Poses[body.Id].Angle
        Assert.True(abs (atan2 (sin difference) (cos difference)) < 1e-6)
    let feet = frames |> List.map (fun frame -> (Motion.footPositions graph frame).Values |> Seq.head)
    Assert.True((feet |> List.maxBy _.X).X - (feet |> List.minBy _.X).X > 10.0)
    Assert.True((feet |> List.maxBy _.Y).Y - (feet |> List.minBy _.Y).Y > 5.0)
    for a, b in List.pairwise feet do Assert.True(Vec3.distance a b < 8.0, "Foot path jumped assembly branch.")

[<Fact>]
let ``generic four bar agrees with an independent analytic oracle at all phases`` () =
    for n in 0..72 do
        let angle = float n*Math.PI/36.0
        let program = MotionExamples.fourBar Genome.defaults angle |> Result.defaultWith failwith
        let graph = compiled program
        let frame = solved graph angle None
        let b = Vec3.create (20.0*cos angle) (20.0*sin angle) 0
        let delta = Vec3.subtract (Vec3.create 80 0 0) b
        let distance = Vec3.length delta
        let u = Vec3.scale (1.0/distance) delta
        let along = distance/2.0
        let height = sqrt (80.0*80.0-along*along)
        let joint = Vec3.add b (Vec3.add (Vec3.scale along u) (Vec3.scale height (Vec3.create -u.Y u.X 0)))
        let foot = Vec3.add b (Vec3.scale 1.25 (Vec3.subtract joint b))
        Assert.True(Vec3.distance b (position graph frame "crank" "tip") < 1e-7)
        Assert.True(Vec3.distance joint (position graph frame "coupler" "joint") < 1e-7)
        Assert.True(Vec3.distance foot (position graph frame "coupler" "foot") < 1e-7)

[<Theory>]
[<InlineData(1.0)>]
[<InlineData(-1.0)>]
let ``Jansen rigid triangles follow the canonical toe curve in both crank directions`` (direction: float) =
    let program = MotionExamples.named "jansen" |> Result.defaultWith failwith
    let graph = compiled program
    let frames = Motion.sample graph [for n in 0..720 -> direction*float n*Math.PI/180.0]
                 |> Result.defaultWith failwith |> List.toArray
    // Independent circle-intersection reference in Y-down coordinates; no prescribed toe trajectory.
    let expected = [
        0,-43.160110524104724,91.75693292612321
        90,-70.67056317652096,89.6428368009199
        180,-33.72972953816961,73.51709740981946
        270,-7.689066230641672,90.38935136740429
    ]
    for turn in 0..1 do
        for phase,x,y in expected do
            let index = turn*360 + (if direction>0.0 then phase else (360-phase)%360)
            let actual = position graph frames[index] "foot-triangle" "toe"
            Assert.True(Vec3.distance actual (Vec3.create x y 0) < 1e-5)
    let spans = [
        "crank","root","tip",15.0
        "upper-drive","crank","rocker",50.0
        "lower-drive","crank","rocker",61.9
        "upper-triangle","pivot","drive",41.5
        "upper-triangle","pivot","knee",40.1
        "upper-triangle","drive","knee",55.8
        "rocker","root","tip",39.3
        "shin","upper","lower",39.4
        "foot-triangle","pivot","knee",36.7
        "foot-triangle","knee","toe",65.7
        "foot-triangle","pivot","toe",49.0
    ]
    for frame in frames do
        Assert.True(frame.MaxResidual <= 1e-7)
        for body,a,b,length in spans do
            Assert.Equal(length,Vec3.distance (position graph frame body a) (position graph frame body b),6)
    let feet = frames[0..359] |> Array.map (fun frame -> position graph frame "foot-triangle" "toe")
    let low = feet |> Array.maxBy _.Y |> _.Y
    let stance = feet |> Array.filter (fun p -> p.Y > low-1.0)
    let span (points: Vec3 array) = (points |> Array.maxBy _.X).X - (points |> Array.minBy _.X).X
    Assert.InRange(span feet,67.8,68.1)
    Assert.InRange(low-(feet |> Array.minBy _.Y).Y,22.3,22.6)
    Assert.InRange(stance.Length,150,160)
    Assert.InRange(span stance,56.5,58.0)

[<Fact>]
let ``multi turn Jansen cycle diagram shows the labeled first revolution quarters`` () =
    let graph = compiled MotionExamples.jansen
    let frames = Motion.sample graph [for n in 0..720 -> float n*Math.PI/180.0] |> Result.defaultWith failwith
    let svg = MotionExport.cycleSvg MotionExamples.jansen graph frames
    let crank = Regex.Matches(svg, """<polyline points="([^"]+)" fill="none" stroke="#b97800" """)
    Assert.Equal(4,crank.Count)
    let vectors = [for item in crank do
                       let points = item.Groups[1].Value.Split(' ') |> Array.map (fun pair ->
                           pair.Split(',') |> Array.map (fun number -> Double.Parse(number,CultureInfo.InvariantCulture)))
                       yield points[1][0]-points[0][0],points[1][1]-points[0][1]]
    for (x,y),(dx,dy) in List.zip vectors [1,0;0,1;-1,0;0,-1] do
        Assert.True(x*float dx+y*float dy > 1.0)
        Assert.True(abs(x*float dy-y*float dx) < 0.01)

[<Fact>]
let ``Jansen cycle diagram keeps both rigid triangles closed`` () =
    let graph = compiled MotionExamples.jansen
    let frames = Motion.sample graph [for n in 0..36 -> float n*Math.PI/18.0] |> Result.defaultWith failwith
    let svg = MotionExport.cycleSvg MotionExamples.jansen graph frames
    for color in ["#8454bb"; "#786457"] do
        let edges = Regex.Matches(svg, $"""<polyline points="([^"]+)" fill="none" stroke="{color}" """)
        Assert.Equal(12,edges.Count)
        for panel in 0..3 do
            let endpoints = [for n in 0..2 -> edges[panel*3+n].Groups[1].Value.Split(' ')]
            Assert.Equal(endpoints[0][0],endpoints[1][0])
            Assert.Equal(endpoints[0][1],endpoints[2][0])
            Assert.Equal(endpoints[1][1],endpoints[2][1])

[<Fact>]
let ``walker1 dimensions and separate foot are attachment sites not special motion operators`` () =
    let p = MotionExamples.walker1
    let body id = p.Bodies |> List.find (fun b -> b.Id = id)
    let span id a b =
        let sites = (body id).Sites |> List.map (fun s -> s.Id, s.Position) |> Map.ofList
        Vec3.distance sites[a] sites[b]
    Assert.Equal(20.0, span "crank" "root" "tip")
    Assert.Equal(40.0, span "upper" "root" "tip")
    Assert.Equal(80.0, span "crossbar" "left" "right")
    Assert.Equal(80.0, span "long-leg" "root" "drive")
    Assert.Equal(200.0, span "long-leg" "root" "tip")
    Assert.Equal(120.0, span "short-leg" "root" "tip")
    Assert.Equal("sole", p.Feet.Head.Port.Body)
    Assert.Equal(40.0, span "sole" "left" "right")

[<Fact>]
let ``transmission shares one input and reverses an independently supported output`` () =
    let p = {
        Bodies = [
            Motion.ground "ground" [Motion.site "a" 0 0; Motion.site "b" 60 0]
            Motion.rigid "a" 0 0 0 [Motion.site "root" 0 0; Motion.site "tip" 20 0]
            Motion.rigid "b" 60 0 0 [Motion.site "root" 0 0; Motion.site "tip" 20 0]
        ]
        Hinges = [
            Motion.couple "a" (Motion.port "ground" "a") (Motion.port "a" "root")
            Motion.couple "b" (Motion.port "ground" "b") (Motion.port "b" "root")
        ]
        Motor = { Body = "a"; Phase = 0 }
        Transmissions = [Motion.transmit "a" "b" -0.5 0]
        Feet = [Motion.foot "output" "b" "tip" 4]
    }
    let graph = compiled p
    let frame = solved graph Math.PI None
    Assert.Equal(Math.PI, frame.Poses["a"].Angle, 7)
    Assert.Equal(-Math.PI/2.0, frame.Poses["b"].Angle, 7)
    Assert.True(Vec3.distance (Vec3.create 60 -20 0) (position graph frame "b" "tip") < 1e-6)

[<Fact>]
let ``invalid programs and unresolved mobility fail explicitly`` () =
    let p = MotionExamples.walker1
    for invalid in [
        { p with Bodies = p.Bodies @ [p.Bodies.Head] }
        { p with Motor = { Body = "missing"; Phase = 0 } }
        { p with Motor = { Body = "ground"; Phase = 0 } }
        { p with Feet = [Motion.foot "bad" "sole" "missing" 4] }
        { p with Feet = [Motion.foot "bad" "ground" "upper" 4] }
        { p with Hinges = p.Hinges.Tail }
        { p with Bodies = p.Bodies @ [Motion.rigid "disconnected" 100 100 0 [Motion.site "root" 0 0]] }
        { p with Transmissions = [Motion.transmit "crank" "upper" Double.NaN 0] }
    ] do Assert.True((Motion.compile invalid).IsError)
    let graph = compiled p
    Assert.True((Motion.solve graph Double.NaN None).IsError)
    Assert.True((Motion.solve graph 1000.0 None).IsError)

[<Fact>]
let ``overconstrained geometry cannot masquerade as a valid trajectory`` () =
    let p = MotionExamples.fourBar Genome.defaults 0 |> Result.defaultWith failwith
    let lock = Motion.couple "locked-foot" (Motion.port "ground" "drive") (Motion.port "coupler" "foot")
    let impossible = { p with Hinges = p.Hinges @ [lock] }
    let graph = compiled impossible
    Assert.True((Motion.solve graph 0 None).IsError)

[<Fact>]
let ``duplicate constraints do not hide a singular uncontrolled body`` () =
    let p = MotionExamples.fourBar Genome.defaults 0 |> Result.defaultWith failwith
    let free = Motion.rigid "free" 0 0 0 [Motion.site "root" 0 0]
    let hinge id = Motion.couple id (Motion.port "ground" "drive") (Motion.port "free" "root")
    let graph = compiled { p with Bodies = p.Bodies @ [free]; Hinges = p.Hinges @ [hinge "free-a"; hinge "free-b"] }
    match Motion.solve graph 0 None with
    | Error error -> Assert.Contains("Singular", error)
    | Ok _ -> failwith "Duplicate hinges must not resolve free rotation."

[<Fact>]
let ``motor phase and continuation choose a stable branch across reverse sampling`` () =
    let p = MotionExamples.fourBar Genome.defaults 0 |> Result.defaultWith failwith
    let graph = compiled { p with Motor = { p.Motor with Phase = Math.PI/2.0 } }
    let forward = solved graph Math.PI None
    let reverse = solved graph 0 (Some forward)
    Assert.Equal(Math.PI/2.0, reverse.Poses["crank"].Angle, 7)
    Assert.True(reverse.MaxResidual < 1e-7)

[<Fact>]
let ``a topology rewrite adds a closed motion shaping branch without another motor`` () =
    let original = MotionExamples.fourBar Genome.defaults 0 |> Result.defaultWith failwith
    let changed =
        Motion.addDyad "branch" (Motion.port "ground" "drive") (Motion.port "coupler" "foot") 180 180 1 original
        |> Result.defaultWith failwith
    Assert.Equal(original.Bodies.Length+2, changed.Bodies.Length)
    Assert.Equal(original.Hinges.Length+3, changed.Hinges.Length)
    Assert.Equal(original.Motor, changed.Motor)
    let changed = { changed with Feet = [Motion.foot "new-tip" "branch-a" "tip" 4] }
    let graph = compiled changed
    let frames = cycle graph
    Assert.True(frames |> List.forall (fun frame -> frame.MaxResidual < 1e-7))
    let before = compiled original |> cycle
    let originalFoot = (Motion.footPositions (compiled original) before.Head)["coupler-tip"]
    let changedFoot = (Motion.footPositions graph frames.Head)["new-tip"]
    Assert.True(Vec3.distance originalFoot changedFoot > 20.0)
    Assert.True((Motion.addDyad "bad" (Motion.port "ground" "drive") (Motion.port "coupler" "foot") 1 1 1 original).IsError)
    Assert.True((Motion.addDyad "bad" (Motion.port "ground" "drive") (Motion.port "coupler" "foot") 180 180 0 original).IsError)

[<Fact>]
let ``initially closing construction with an impossible cycle is not exported as success`` () =
    let path = Path.Combine(Path.GetTempPath(), $"lego-motion-invalid-{Guid.NewGuid():N}")
    let original = MotionExamples.fourBar Genome.defaults 0 |> Result.defaultWith failwith
    let changed =
        Motion.addDyad "short-branch" (Motion.port "ground" "drive") (Motion.port "coupler" "foot") 60 50 1 original
        |> Result.defaultWith failwith
    Assert.Throws<ArgumentException>(fun () -> MotionExport.write path changed 72 |> ignore) |> ignore
    Assert.False(Directory.Exists path)

[<Fact>]
let ``multi turn motion exports cover the full requested motor interval`` () =
    let path = Path.Combine(Path.GetTempPath(), $"lego-motion-turns-{Guid.NewGuid():N}")
    try
        let program = MotionExamples.fourBar Genome.defaults 0 |> Result.defaultWith failwith
        let _, count = MotionExport.writeTurns path program 36 2
        Assert.Equal(73, count)
        use doc = JsonDocument.Parse(File.ReadAllText(Path.Combine(path, "motion.json")))
        let frames = doc.RootElement.GetProperty("frames")
        Assert.Equal(73, frames.GetArrayLength())
        Assert.True(abs (frames.[72].GetProperty("inputAngle").GetDouble()-4.0*Math.PI) < 1e-10)
        Assert.Equal(program, MotionExport.readProgram (Path.Combine(path, "program.json")))
    finally
        if Directory.Exists path then Directory.Delete(path, true)

[<Fact>]
let ``motion programs round trip and exported previews contain solved trajectories`` () =
    let path = Path.Combine(Path.GetTempPath(), $"lego-motion-{Guid.NewGuid():N}")
    try
        let program = MotionExamples.walker1
        let _, count = MotionExport.write path program 72
        Assert.Equal(73, count)
        Assert.Equal(program, MotionExport.readProgram (Path.Combine(path, "program.json")))
        use doc = JsonDocument.Parse(File.ReadAllText(Path.Combine(path, "motion.json")))
        Assert.Equal("planar-kinematics-not-dynamics", doc.RootElement.GetProperty("mode").GetString())
        Assert.Equal(73, doc.RootElement.GetProperty("frames").GetArrayLength())
        let first = doc.RootElement.GetProperty("frames").[0]
        Assert.Equal(8, first.GetProperty("poses").GetArrayLength())
        let html = File.ReadAllText(Path.Combine(path, "preview.html"))
        Assert.DoesNotContain("__MOTION_JSON__", html)
        Assert.Contains("Kinematics only", html)
        Assert.Throws<ArgumentException>(fun () -> MotionExport.write path program 72 |> ignore) |> ignore
        let json = MotionExport.programJson program
        for invalid in [
            json.Replace("\"hinges\":", "\"unknown\":")
            json.Replace("\"feet\": [", "\"feet\": null, \"unused\": [")
            json.Replace("\"phase\": 0", "\"phaseTypo\": 0")
            "{\"schemaVersion\":1,\"program\":null}"
        ] |> List.distinct do
            if invalid <> json then
                File.WriteAllText(Path.Combine(path, "invalid.json"), invalid)
                Assert.ThrowsAny<ArgumentException>(fun () -> MotionExport.readProgram (Path.Combine(path, "invalid.json")) |> ignore) |> ignore
    finally
        for name in ["program.json"; "motion.json"; "preview.html"; "cycle.svg"; "invalid.json"] do
            let file = Path.Combine(path, name)
            if File.Exists file then File.Delete file
        if Directory.Exists path then Directory.Delete path

[<Fact>]
let ``readme regression images are reproducible outputs of the tested programs`` () =
    let path = Path.Combine(Path.GetTempPath(), $"lego-motion-gallery-{Guid.NewGuid():N}")
    let names = ["four-bar.svg"; "walker1.svg"; "added-dyad.svg"]
    try
        Assert.Equal<string list>(names, MotionExport.writeTestGallery path)
        for name in names do
            let actual = File.ReadAllText(Path.Combine(path, name))
            let expected = File.ReadAllText(Path.Combine(AppContext.BaseDirectory, "visualizations", name))
            Assert.Equal(expected.Replace("\r\n", "\n"), actual.Replace("\r\n", "\n"))
            let svg = System.Xml.Linq.XDocument.Parse actual
            Assert.Equal("svg", svg.Root.Name.LocalName)
            Assert.DoesNotContain("<script", actual)
            Assert.Contains("foot trajectories", actual)
    finally
        for name in names do
            let file = Path.Combine(path, name)
            if File.Exists file then File.Delete file
        if Directory.Exists path then Directory.Delete path
