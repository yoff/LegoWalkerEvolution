module LegoWalkers.RealizationTests

open System
open System.IO
open System.Text.Json
open LegoWalkers
open Xunit

let screened name =
    MotionExamples.named name |> Result.defaultWith failwith
    |> fun p -> Realization.screen p 72
    |> Result.defaultWith (fun g -> failwith (Realization.json g))

let proposed s = Realization.propose s 4 |> Result.defaultWith (fun g -> failwith (Realization.json g))
let realized s g = Realization.compile s g |> Result.defaultWith (fun g -> failwith (Realization.json g))

[<Fact>]
let ``one hole raised anchor accepts solver rounding but not off-grid dimensions`` () =
    let source = MotionExamples.fourBar {Genome.defaults with GroundHoles=2;CouplerHoles=4;RockerHoles=4} 0.0
                 |> Result.defaultWith failwith
    let bodies =
        source.Bodies |> List.map (fun body ->
            if body.Grounded then
                let sites = body.Sites |> List.map (fun site ->
                    if site.Id="drive" then {site with Position={site.Position with Y= -1e-14}}
                    elif site.Id="rocker" then {site with Position={site.Position with Y= -20.0}} else site)
                {body with Sites=sites}
            else body)
    let program = {source with Bodies=bodies}
    let s = Realization.screen program 72 |> Result.defaultWith (Realization.json >> failwith)
    Assert.NotEmpty (proposed s)
    let offGridBodies =
        program.Bodies |> List.map (fun body ->
            if body.Grounded then
                let sites = body.Sites |> List.map (fun site ->
                    if site.Id="rocker" then {site with Position={site.Position with Y= -19.0}} else site)
                {body with Sites=sites}
            else body)
    let offGrid = {program with Bodies=offGridBodies}
    let offGrid = Realization.screen offGrid 72 |> Result.defaultWith (Realization.json >> failwith)
    Assert.True(Result.isError (Realization.propose offGrid 1))

[<Theory>]
[<InlineData(40.0)>]
[<InlineData(-40.0)>]
let ``native molded L fits both orientations with exact proper socket transforms`` toe =
    let spec = Catalog.get "32526.dat"
    Assert.Equal(7, spec.Ports.Length)
    let points = [Vec3.create 0 0 40; Vec3.create 0 -60 40; Vec3.create toe 0 40]
    let matches = Catalog.fitPlanarPart spec.Id (Vec3.create 0 0 1) points
    Assert.NotEmpty matches
    for position,rotation in matches do
        Assert.True(Rotation.valid rotation)
        for point in points do
            Assert.Contains(spec.Ports, fun port ->
                Vec3.distance point (Vec3.add position (Rotation.apply rotation port.Position)) < 1e-6
                && abs (abs (Rotation.apply rotation port.Axis).Z-1.0) < 1e-8)
    Assert.Equal<(Vec3 * Rotation) list>(matches, Catalog.fitPlanarPart spec.Id (Vec3.create 0 0 1) points)

[<Fact>]
let ``socket fitting never stretches parts or accepts invalid geometry`` () =
    let axis = Vec3.create 0 0 1
    Assert.Empty(Catalog.fitPlanarPart "32526.dat" axis [Vec3.zero;Vec3.create 41 0 0])
    Assert.Throws<ArgumentException>(fun () -> Catalog.fitPlanarPart "32526.dat" Vec3.zero [Vec3.zero;Vec3.create 40 0 0] |> ignore) |> ignore
    Assert.Throws<ArgumentException>(fun () -> Catalog.fitPlanarPart "32526.dat" axis [Vec3.zero] |> ignore) |> ignore
    Assert.Throws<ArgumentException>(fun () -> Catalog.fitPlanarPart "32526.dat" axis [Vec3.zero;Vec3.create 40 0 1] |> ignore) |> ignore
    Assert.Throws<ArgumentException>(fun () -> Catalog.fitPlanarPart "32526.dat" axis [Vec3.zero;Vec3.create Double.NaN 0 0] |> ignore) |> ignore

[<Theory>]
[<InlineData(40.0)>]
[<InlineData(-40.0)>]
let ``reusable reference leg lowers through native L frame and mirrored shoe recipes`` toe =
    let p = MotionExamples.doubleParallelogram 60 toe |> Result.defaultWith failwith
    let s = Realization.screen p 72 |> Result.defaultWith (Realization.json >> failwith)
    let variants = proposed s |> List.take 2
    for genome in variants do
        Assert.Equal(2, (genome.Bodies |> List.find (fun b -> b.Body=p.Motor.Body)).Layer)
        Assert.Equal("44294.dat", (genome.Connections |> List.find (fun c -> c.Group="bearing")).Part)
        let r = realized s genome
        Assert.Equal("32526.dat", (r.Assembly.Parts |> List.find (fun p -> p.Body=Some "upright-l")).Part)
        Assert.Equal("32525.dat", (r.Assembly.Parts |> List.find (fun p -> p.Id="frame-upright-0")).Part)
        Assert.Equal(10, r.Assembly.Joints.Length)
        Assert.Equal(4, r.Assembly.Feet.Length)
        Assert.Equal(2, (r.Groups |> List.find (fun (_,_,bodies) -> bodies.Length=3) |> fun (_,hinges,_) -> hinges.Length))
        Assert.Equal(Export.ldraw r.Assembly, Export.ldraw (realized s genome).Assembly)

[<Theory>]
[<InlineData("four-bar", 4, 4)>]
[<InlineData("walker1", 8, 10)>]
let ``known mechanisms lower through common part socket and layer recipes`` name bodies hinges =
    let s = screened name
    let variants = proposed s
    Assert.Equal(4, variants.Length)
    let assemblies = variants |> List.map (realized s)
    Assert.Equal(4, assemblies |> List.map _.AssemblyId |> Set.ofList |> Set.count)
    for r in assemblies do
        Assert.Equal(s.MechanismId, r.Genome.MechanismId)
        Assert.Equal(bodies, r.Assembly.Parts |> List.choose _.Body |> Set.ofList |> Set.count)
        Assert.Equal(hinges, r.Assembly.Joints.Length)
        Assert.Single(r.Assembly.Drives) |> ignore
        Assert.All(r.Assembly.Parts, fun p ->
            Catalog.get p.Part |> ignore
            Assert.True(Rotation.valid p.Rotation))
        Assert.NotEqual<string>("reject", (Realization.screenGeometry s r).Outcome)
        let again = realized s r.Genome
        Assert.Equal(Export.ldraw r.Assembly, Export.ldraw again.Assembly)
        Assert.Equal(r.GenotypeId, again.GenotypeId)
        Assert.Equal(r.AssemblyId, again.AssemblyId)

[<Fact>]
let ``shared pins preserve all abstract hinges without overlapping connectors`` () =
    let s = screened "walker1"
    let r = realized s (proposed s).Head
    let shared = r.Groups |> List.filter (fun (_, _, bodies) -> bodies.Length = 3)
    Assert.Equal(2, shared.Length)
    for id, hinges, bodies in shared do
        Assert.Equal(2, hinges.Length)
        let joints = r.Assembly.Joints |> List.filter (fun j -> j.Connector = $"pivot-{id}")
        Assert.Equal(2, joints.Length)
        Assert.Equal<Set<string>>(Set.ofList bodies, joints |> List.collect (fun j -> [j.BodyA; j.BodyB]) |> Set.ofList)
        let pins = r.Assembly.Parts |> List.filter (fun p -> p.Id = $"pivot-{id}")
        Assert.Equal("32556.dat", (Assert.Single pins).Part)
    Assert.Equal(4, r.Assembly.Feet.Length)
    Assert.All(r.Assembly.Feet, fun f -> Assert.Equal("body-sole", f.Part))

[<Fact>]
let ``choice ordering does not affect decoding or identity`` () =
    let s = screened "walker1"
    let g = (proposed s).Head
    let a = realized s g
    let b = realized s { g with Bodies = List.rev g.Bodies; Connections = List.rev g.Connections }
    Assert.Equal(a.GenotypeId, b.GenotypeId)
    Assert.Equal(a.AssemblyId, b.AssemblyId)
    Assert.Equal(Export.ldraw a.Assembly, Export.ldraw b.Assembly)
    let reordered =
        { s.Program with Bodies = List.rev s.Program.Bodies; Hinges = List.rev s.Program.Hinges }
    Assert.Equal(s.MechanismId, Realization.mechanismId reordered)
    let other = Realization.screen reordered 72 |> Result.defaultWith (fun g -> failwith (Realization.json g))
    let c = realized other g
    Assert.Equal(a.AssemblyId, c.AssemblyId)
    Assert.Equal(Export.ldraw a.Assembly, Export.ldraw c.Assembly)

[<Fact>]
let ``full genomes need not map one to one to exported physical assemblies`` () =
    let s = screened "four-bar"
    let g = (proposed s).Head
    let altered = { g with Bodies = g.Bodies |> List.map (fun b ->
                        if b.Body = "crank" then { b with Offset = b.Offset+1e-11 } else b) }
    let a, b = realized s g, realized s altered
    Assert.NotEqual<string>(a.GenotypeId, b.GenotypeId)
    Assert.Equal(a.AssemblyId, b.AssemblyId)

[<Fact>]
let ``bad realization choices reject only that concrete realization`` () =
    let s = screened "four-bar"
    let g = (proposed s).Head
    let bad = [
        { g with Side = 0 }
        { g with RecipeVersion = "other" }
        { g with MechanismId = String.replicate 64 "0" }
        { g with Bodies = g.Bodies.Tail }
        { g with Bodies = g.Bodies |> List.map (fun b -> { b with Layer = 0 }) }
        { g with Connections = g.Connections |> List.map (fun c -> { c with Center = 10000.0 }) }
    ]
    for genome in bad do
        match Realization.compile s genome with
        | Ok _ -> failwith "An invalid choice was accepted."
        | Error gate -> Assert.Equal("lego-realizability", gate.Gate)
    Assert.True((Realization.compile s g).IsOk)

[<Fact>]
let ``motion screening rejects structural errors and leaves gait preferences as warnings`` () =
    let p = (screened "four-bar").Program
    let invalid = { p with Hinges = { p.Hinges.Head with A = Motion.port "missing" "socket" } :: p.Hinges.Tail }
    match Realization.screen invalid 72 with
    | Ok _ -> failwith "Bad graph passed."
    | Error gate -> Assert.Equal("structure", gate.Gate)
    let stationary = { p with Feet = [Motion.foot "fixed-output" "crank" "root" 4] }
    let s = Realization.screen stationary 72 |> Result.defaultWith (fun g -> failwith (Realization.json g))
    let potential = s.Decisions |> List.find (fun g -> g.Gate = "motion-potential")
    Assert.Equal("warning", potential.Outcome)
    for frame in s.Frames do
        let feet = Motion.footPositions s.Graph frame
        Assert.True(Vec3.length feet["fixed-output"] < 1e-6)
    Assert.Throws<ArgumentException>(fun () -> Realization.propose s 33 |> ignore) |> ignore

[<Fact>]
let ``mobility rejects a nonclosing phase but does not exclude longer period transmissions`` () =
    let original = (screened "four-bar").Program
    let impossible =
        Motion.addDyad "short-branch" (Motion.port "ground" "drive") (Motion.port "coupler" "foot") 60 50 1 original
        |> Result.defaultWith failwith
    match Realization.screen impossible 72 with
    | Ok _ -> failwith "An impossible cycle passed."
    | Error gate -> Assert.Equal("mobility", gate.Gate)
    let twoTurn = {
        Bodies = [
            Motion.ground "frame" [Motion.site "input" 0 0; Motion.site "output" 40 0]
            Motion.rigid "input" 0 0 0 [Motion.site "root" 0 0]
            Motion.rigid "output" 40 0 0 [Motion.site "root" 0 0; Motion.site "tip" 20 0]
        ]
        Hinges = [
            Motion.couple "input" (Motion.port "frame" "input") (Motion.port "input" "root")
            Motion.couple "output" (Motion.port "frame" "output") (Motion.port "output" "root")
        ]
        Motor = { Body = "input"; Phase = 0 }
        Transmissions = [Motion.transmit "input" "output" 0.5 0]
        Feet = [Motion.foot "tip" "output" "tip" 4]
    }
    let s = Realization.screen twoTurn 72 |> Result.defaultWith (fun g -> failwith (Realization.json g))
    Assert.Equal("warning", (s.Decisions |> List.find (fun g -> g.Gate = "mobility")).Outcome)
    Assert.True((Realization.propose s 1).IsError)

[<Fact>]
let ``exports preserve the complete genome and ordered zero trial preflight reports`` () =
    let directory = Path.Combine(Path.GetTempPath(), "legowalkers-realization-"+Guid.NewGuid().ToString("N"))
    try
        let s = screened "walker1"
        let r = realized s (proposed s).Head
        Realization.write directory s r |> ignore
        let genome = Realization.readGenome (Path.Combine(directory, "realization.json"))
        Assert.Equal(r.GenotypeId, (realized s genome).GenotypeId)
        use manifest = JsonDocument.Parse(File.ReadAllText(Path.Combine(directory, "manifest.json")))
        let data = manifest.RootElement
        Assert.Equal(Realization.version, data.GetProperty("grammarVersion").GetString())
        Assert.Equal(s.MechanismId, data.GetProperty("genome").GetProperty("mechanismId").GetString())
        Assert.False(data.GetProperty("genome").TryGetProperty("pairs") |> fst)
        Assert.Equal(1, data.GetProperty("motion").GetProperty("footCount").GetInt32())
        Assert.Equal(1, data.GetProperty("motion").GetProperty("logicalFeet").GetArrayLength())
        Assert.Equal(4, data.GetProperty("feet").GetArrayLength())
        for region in data.GetProperty("feet").EnumerateArray() do
            Assert.Equal("sole", region.GetProperty("logicalFoot").GetString())
        Assert.Equal(Export.sha256 (File.ReadAllText(Path.Combine(directory, "model.ldr"))), data.GetProperty("ldrawSha256").GetString())
        use gates = JsonDocument.Parse(File.ReadAllText(Path.Combine(directory, "gates.json")))
        Assert.Equal<string list>(
            ["structure"; "mobility"; "motion-potential"; "lego-realizability"; "geometry"],
            gates.RootElement.EnumerateArray() |> Seq.map (fun g -> g.GetProperty("gate").GetString()) |> Seq.toList)
        File.WriteAllText(Path.Combine(directory, "invalid.json"), """{"schemaVersion":1}""")
        Assert.Throws<ArgumentException>(fun () -> Realization.readGenome (Path.Combine(directory, "invalid.json")) |> ignore) |> ignore
    finally
        if Directory.Exists directory then Directory.Delete(directory, true)

[<Fact>]
let ``logical foot count follows outputs not contact patches and is capped at six`` () =
    let p = (screened "four-bar").Program
    let withFeet count =
        let sites = [for i in 0..count-1 -> Motion.site $"output-{i}" (float i*20.0) 0]
        { p with
            Bodies = p.Bodies |> List.map (fun b -> if b.Id = "coupler" then { b with Sites = b.Sites @ sites } else b)
            Feet = [for i in 0..count-1 -> Motion.foot $"foot-{i}" "coupler" $"output-{i}" 4] }
    let six = withFeet 6
    Assert.Equal(3, Motion.footCount (withFeet 3))
    Assert.True((Motion.compile (withFeet 3)).IsOk)
    Assert.Equal(6, Motion.footCount six)
    Assert.True((Motion.compile six).IsOk)
    match Motion.compile (withFeet 7) with
    | Ok _ -> failwith "Seven feet passed the search limit."
    | Error errors -> Assert.Contains(errors, fun e -> e.Contains("at most 6"))
    let alias =
        { p with Feet = p.Feet @ [{ p.Feet.Head with Id = "duplicate-foot" }] }
    match Motion.compile alias with
    | Ok _ -> failwith "Duplicate foot outputs are not additional feet."
    | Error errors -> Assert.Contains(errors, fun e -> e.Contains("distinct output site"))

[<Fact>]
let ``a lineage can change inferred foot count without changing power or adding towing`` () =
    let parent = { Genome.defaults with Pairs = 1; Remote = true }
    for pairs in [2; 3] do
        let child, operation, fields = Evolution.propose pairs true false 42 0 (Some parent)
        Assert.Equal("change-foot-count", operation)
        Assert.Equal<string list>(["pairs"], fields)
        Assert.True(child.Remote)
        Assert.False(child.TowWagon)
        let assembly = Grammar.compile Walker child |> Result.defaultWith (String.concat "; " >> failwith)
        Assert.Equal(pairs*2, assembly.Feet.Length)
        let again = Evolution.propose pairs true false 42 1 (Some parent)
        Assert.Equal(again, Evolution.propose pairs true false 42 1 (Some parent))
