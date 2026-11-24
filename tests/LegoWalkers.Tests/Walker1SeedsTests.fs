module LegoWalkers.Walker1SeedsTests

open System
open LegoWalkers
open Xunit

let private founder () =
    let program, genome = Walker1Seeds.walker 2
    let s = Realization.screen program 72 |> Result.defaultWith (fun g -> failwith (Realization.json g))
    let r = WalkerRealization.compile s genome |> Result.defaultWith (fun g -> failwith (Realization.json g))
    s, r

[<Fact>]
let ``Walker1 founder preserves two real linkages on a single remote drivetrain`` () =
    let s, r = founder ()
    Assert.Equal(16, s.Program.Bodies.Length)
    Assert.Equal(21, s.Program.Hinges.Length)
    Assert.Equal(2, Motion.footCount s.Program)
    Assert.Equal(8, r.Assembly.Feet.Length)
    Assert.Equal(2, r.LogicalFeet |> Map.values |> Set.ofSeq |> Set.count)
    Assert.Single(r.Assembly.Drives) |> ignore
    Assert.Single(r.Assembly.Parts |> List.filter (fun p -> p.Part = "58121.dat")) |> ignore
    for id, part in ["motor", "58121.dat"; "payload-battery", "58119.dat"; "payload-receiver", "58123.dat"] do
        Assert.Contains(r.Assembly.Parts, fun p -> p.Id = id && p.Part = part && p.Body = Some "chassis")
    Assert.Equal(2, r.Assembly.Gears.Length)
    Assert.Equal(4, r.Groups |> List.filter (fun (_, _, bs) -> bs.Length = 3) |> List.length)
    Assert.DoesNotContain(r.Assembly.Parts, fun p -> p.Id.Contains("input-motor") || p.Id.Contains("frame-rail"))
    Assert.Equal<string list>([], Grammar.validateAssembly r.Assembly)
    Assert.NotEqual<string>("reject", (Realization.screenAssemblyGeometry s r.Assembly).Outcome)
    Assert.All(s.Frames, fun frame -> Assert.True(frame.MaxResidual < 1e-7))
    for side in 0..1 do
        for id in ["upper"; "crossbar"; "brace"; "long-leg"; "short-leg"; "sole"] do
            Assert.Contains(s.Program.Bodies, fun b -> b.Id = $"leg-{side}-{id}")
        let cranks = r.Genome.Bodies |> List.find (fun b -> b.Body = "shaft-2")
        Assert.Contains(cranks.Parts, fun p -> p.Id = $"leg-{side}-body-crank" && p.Part = "60483.dat")
    let outputAxle = r.Genome.Connections |> List.find (fun c -> c.Group = "bearing-2")
    Assert.Equal("3707.dat", outputAxle.Placement.Part)

[<Fact>]
let ``Walker1 sole contacts stay on their actual source bricks`` () =
    let s, r = founder ()
    for foot in s.Program.Feet do
        Assert.Equal(4.0, foot.Radius)
        let choice = r.Genome.Feet |> List.find (fun f -> f.Foot = foot.Id)
        let p = r.Assembly.Parts |> List.find (fun p -> p.Id = choice.Part)
        Assert.Equal("3701.dat", p.Part)
        Assert.Equal(Some foot.Port.Body, p.Body)
        Assert.Equal(p.Position.Z, choice.Depth)
        let regions = r.Assembly.Feet |> List.filter (fun f -> r.LogicalFeet[f.Id] = foot.Id)
        Assert.Equal(4, regions.Length)
        for actual in regions do
            let delta = Vec3.subtract actual.Position p.Position
            let local = Vec3.create (Vec3.dot delta p.Rotation.X) (Vec3.dot delta p.Rotation.Y) (Vec3.dot delta p.Rotation.Z)
            Assert.True(abs(abs local.X-40.0) < 1e-6)
            Assert.True(abs(local.Y-24.0) < 1e-6)
            Assert.True(abs(abs local.Z-10.0) < 1e-6)

[<Fact>]
let ``Walker1 source paths survive composition without fixture hardware duplication`` () =
    let s, _ = founder ()
    let source = Realization.screen MotionExamples.walker1 72 |> Result.defaultWith (fun g -> failwith (Realization.json g))
    for frame, original in List.zip s.Frames source.Frames do
        let expected = (Motion.footPositions source.Graph original)["sole"] |> Vec3.add (Vec3.create 260 0 0)
        for actual in Motion.footPositions s.Graph frame |> Map.values do
            Assert.True(Vec3.distance expected actual < 1e-5)
    let _, again = founder ()
    let _, first = founder ()
    Assert.Equal(first.GenotypeId, again.GenotypeId)
    Assert.Equal(first.AssemblyId, again.AssemblyId)

[<Theory>]
[<InlineData(0)>]
[<InlineData(1)>]
[<InlineData(3)>]
[<InlineData(4)>]
[<InlineData(6)>]
let ``Walker1 recipe rejects unsupported foot counts explicitly`` feet =
    Assert.Throws<ArgumentException>(fun () -> Walker1Seeds.walker feet |> ignore) |> ignore
