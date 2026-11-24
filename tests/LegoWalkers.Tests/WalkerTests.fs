module LegoWalkers.WalkerTests

open System
open System.IO
open System.Text.Json
open LegoWalkers
open Xunit

let private native feet =
    let program, genome = WalkerSeeds.fourBar feet
    let s = Realization.screen program 72 |> Result.defaultWith (fun g -> failwith (Realization.json g))
    let r = WalkerRealization.compile s genome |> Result.defaultWith (fun g -> failwith (Realization.json g))
    s, r

[<Fact>]
let ``manual edit catalog is stable and exposes motion and unchanged-motion realization edits`` () =
    let s,r = native 2
    let edits = WalkerEvolution.editsWithPolicy WalkerBuilder.defaultPolicy s r.Genome
    let again = WalkerEvolution.editsWithPolicy WalkerBuilder.defaultPolicy s r.Genome
    Assert.Equal<string list>(edits |> List.map (fun (id,_,_) -> id), again |> List.map (fun (id,_,_) -> id))
    Assert.Equal(edits.Length, edits |> List.map (fun (id,_,_) -> id) |> Set.ofList |> Set.count)
    Assert.Contains(edits, fun (_,level,_) -> level = "motion")
    Assert.Contains(edits, fun (_,level,_) -> level = "realization")
    Assert.Contains(edits, fun (_,level,_) -> level = "coupled")
    for id,level,mutation in edits do
        Assert.Equal(64, id.Length)
        if level = "realization" then Assert.Equal(s.Program, mutation.Program)

[<Theory>]
[<InlineData(2)>]
[<InlineData(3)>]
[<InlineData(4)>]
[<InlineData(5)>]
[<InlineData(6)>]
let ``whole walkers infer even and odd feet and physically branch one motor`` feet =
    let s, r = native feet
    Assert.Equal(feet, Motion.footCount s.Program)
    Assert.Equal(2*feet, r.Assembly.Feet.Length)
    for logical in s.Program.Feet do
        let choice = r.Genome.Feet |> List.find (fun f -> f.Foot = logical.Id)
        let source = r.Assembly.Parts |> List.find (fun p -> p.Id = choice.Part)
        let anchors = r.Assembly.Feet |> List.filter (fun f -> r.LogicalFeet[f.Id] = logical.Id)
        Assert.Equal(2, anchors.Length)
        let local = anchors |> List.map (fun f -> Vec3.dot (Vec3.subtract f.Position source.Position) source.Rotation.Y)
        let half = if source.Part = "32063.dat" then 5.0 else 10.0
        Assert.Equal<float list>([-half;half], local |> List.map (fun x -> Math.Round(x, 6)) |> List.sort)
    Assert.Equal(Walker, r.Assembly.Kind)
    Assert.Single(r.Assembly.Drives) |> ignore
    Assert.Equal(s.Program.Transmissions.Length, r.Assembly.Gears.Length)
    Assert.True(r.Assembly.Gears.Length > 0)
    Assert.Equal(s.Program.Hinges.Length, r.Assembly.Joints.Length)
    Assert.Equal(s.Program.Bodies.Length, r.Genome.Bodies.Length)
    Assert.Contains(r.Assembly.Parts, fun p -> p.Id = "payload-battery" && p.Body = Some r.Assembly.Chassis)
    Assert.Contains(r.Assembly.Parts, fun p -> p.Id = "payload-receiver" && p.Body = Some r.Assembly.Chassis)
    Assert.NotEqual<string>("reject", (Realization.screenAssemblyGeometry s r.Assembly).Outcome)
    Assert.True(s.Frames |> List.forall (fun frame -> frame.MaxResidual < 1e-7))

[<Fact>]
let ``whole walker lowering preserves the reference hardware without compiling paired genomes`` () =
    let _, r = native 6
    let original = Grammar.compile Walker { Genome.defaults with Remote = true } |> Result.defaultWith (String.concat "; " >> failwith)
    Assert.Equal(original.Parts.Length, r.Assembly.Parts.Length)
    let byId =
        original.Parts |> List.map (fun p ->
            let id =
                match original.Joints |> List.tryFind (fun j -> j.Connector = p.Id) with
                | Some j -> $"pivot-{j.Id}"
                | None -> p.Id
            id, p) |> Map.ofList
    for p in r.Assembly.Parts do
        let expected = byId[p.Id]
        Assert.Equal(expected.Part, p.Part)
        Assert.Equal(expected.Body, p.Body)
        Assert.True(Vec3.distance expected.Position p.Position < 1e-6, p.Id)
        Assert.True(Vec3.distance expected.Rotation.X p.Rotation.X < 1e-6, p.Id)
        Assert.True(Vec3.distance expected.Rotation.Y p.Rotation.Y < 1e-6, p.Id)

[<Fact>]
let ``whole walker choices round trip and ordering does not change identities`` () =
    let s, r = native 3
    let directory = Path.Combine(Path.GetTempPath(), Guid.NewGuid().ToString("N"))
    try
        WalkerRealization.write directory s r |> ignore
        let g = WalkerRealization.readGenome (Path.Combine(directory, "realization.json"))
        let reordered = { g with Bodies = g.Bodies |> List.rev |> List.map (fun b -> { b with Parts = List.rev b.Parts })
                                 Connections = List.rev g.Connections; Mounts = List.rev g.Mounts; Feet = List.rev g.Feet }
        let again = WalkerRealization.compile s reordered |> Result.defaultWith (fun g -> failwith (Realization.json g))
        Assert.Equal(r.GenotypeId, again.GenotypeId)
        Assert.Equal(r.AssemblyId, again.AssemblyId)
        Assert.Equal(Export.ldraw r.Assembly, Export.ldraw again.Assembly)
        use doc = JsonDocument.Parse(File.ReadAllText(Path.Combine(directory, "manifest.json")))
        Assert.Equal("technic-motion-v3", doc.RootElement.GetProperty("grammarVersion").GetString())
        Assert.Equal(3, doc.RootElement.GetProperty("motion").GetProperty("footCount").GetInt32())
        for foot in doc.RootElement.GetProperty("motion").GetProperty("logicalFeet").EnumerateArray() do
            Assert.Equal("capsule", foot.GetProperty("contactShape").GetString())
            Assert.Equal(2, foot.GetProperty("regions").GetArrayLength())
        Assert.False(doc.RootElement.GetProperty("genome").TryGetProperty("pairs") |> fst)
        File.WriteAllText(Path.Combine(directory, "missing.json"), """{"schemaVersion":1}""")
        Assert.Throws<JsonException>(fun () -> WalkerRealization.readGenome (Path.Combine(directory, "missing.json")) |> ignore) |> ignore
    finally
        if Directory.Exists directory then Directory.Delete(directory, true)

[<Fact>]
let ``unsupported or incomplete whole walker hardware rejects explicitly`` () =
    let s, r = native 4
    let g = r.Genome
    let invalid = [
        { g with Remote = false }; { g with TowWagon = true }
        { g with Bodies = g.Bodies.Tail }; { g with Connections = g.Connections.Tail }
        { g with MotorPart = "missing" }; { g with MaxTorque = 0.0 }
        { g with Mounts = { g.Mounts.Head with Part = "3673.dat" } :: g.Mounts.Tail }
    ]
    for genome in invalid do Assert.True((WalkerRealization.compile s genome).IsError)
    let one = { s.Program with Feet = [s.Program.Feet.Head] }
    let screened = Realization.screen one 72 |> Result.defaultWith (fun g -> failwith (Realization.json g))
    Assert.True((WalkerRealization.compile screened { g with MechanismId = screened.MechanismId }).IsError)
    let reduction = { s.Program with Transmissions = { s.Program.Transmissions.Head with Ratio = -0.5 } :: s.Program.Transmissions.Tail }
    let screened = Realization.screen reduction 72 |> Result.defaultWith (fun g -> failwith (Realization.json g))
    Assert.True((WalkerRealization.compile screened { g with MechanismId = screened.MechanismId }).IsError)
    Assert.Throws<ArgumentException>(fun () -> WalkerSeeds.fourBar 1 |> ignore) |> ignore
    Assert.Throws<ArgumentException>(fun () -> WalkerSeeds.fourBar 7 |> ignore) |> ignore

[<Fact>]
let ``whole walker motion is an inspectable gallery snapshot`` () =
    let program, _ = WalkerSeeds.fourBar 4
    let s = Realization.screen program 180 |> Result.defaultWith (fun g -> failwith (Realization.json g))
    let svg = MotionExport.cycleSvg program s.Graph s.Frames
    let snapshot = Path.Combine(AppContext.BaseDirectory, "visualizations", "whole-walker.svg")
    Assert.Equal(File.ReadAllText(snapshot).Replace("\r\n", "\n"), svg.Replace("\r\n", "\n"))
