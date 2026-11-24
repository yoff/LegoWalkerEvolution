module LegoWalkers.Tests

open System
open System.Globalization
open System.IO
open System.Text.Json
open LegoWalkers
open Xunit

let compile kind genome =
    match Grammar.compile kind genome with
    | Ok result -> result
    | Error errors -> failwith (String.concat "; " errors)

[<Fact>]
let ``default four bar closes at every sampled angle`` () =
    Assert.Empty(Kinematics.validate Genome.defaults)
    for n in 0..719 do
        let pose =
            match Kinematics.solve Genome.defaults (float n*Math.PI/360.0) with
            | Ok pose -> pose
            | Error error -> failwith error
        Assert.Equal(20.0, Vec3.distance pose.A pose.B, 7)
        Assert.Equal(80.0, Vec3.distance pose.B pose.C, 7)
        Assert.Equal(80.0, Vec3.distance pose.D pose.C, 7)
        Assert.Equal(100.0, Vec3.distance pose.B pose.Foot, 7)

[<Fact>]
let ``remote mount genes produce connected pin recipes for all positions`` () =
    for pairs in 1..3 do
        for battery in 0..2 do
            for level in 0..1 do
                for receiver in 0..2 do
                    for side in 0..1 do
                        let g = { Genome.defaults with Pairs = pairs; Remote = true; BatterySlot = battery
                                                       BatteryLevel = level; ReceiverSlot = receiver; ReceiverSide = side }
                        let a = compile Walker g
                        let part id = a.Parts |> List.find (fun p -> p.Id = id)
                        Assert.Equal("58119.dat", (part "payload-battery").Part)
                        Assert.Equal("58123.dat", (part "payload-receiver").Part)
                        Assert.True((part "payload-battery").Position.Y <= -120.0)
                        Assert.Equal(6, a.Parts |> List.filter (fun p -> p.Id.StartsWith "battery-mount-" || p.Id.StartsWith "receiver-mount-") |> List.length)
                        Assert.Equal(8+2*level, a.Parts |> List.filter (fun p -> p.Id.StartsWith "battery-support-") |> List.length)
                        Assert.All(a.Parts, fun p -> Assert.True(Rotation.valid p.Rotation))
    let wired = compile Walker Genome.defaults
    Assert.DoesNotContain(wired.Parts, fun p -> p.Id.StartsWith "payload-")
    for kind in [HingeFixture; RigidFixture; MotorFixture] do
        Assert.True((Grammar.compile kind {Genome.defaults with Remote = true}).IsError)
    let reversedMotor = compile MotorFixture {Genome.defaults with MotorDirection = -1}
    Assert.Equal(-3.0, reversedMotor.Drives.Head.Speed)

[<Fact>]
let ``mutations are deterministic single gene descendants within a category`` () =
    for remote in [false; true] do
        let parent = { Genome.defaults with Remote = remote }
        for attempt in 0..15 do
            let first = Evolution.propose 3 remote false 42 attempt (Some parent)
            Assert.Equal(first, Evolution.propose 3 remote false 42 attempt (Some parent))
            let child, operation, changed = first
            Assert.Equal(parent.Pairs, child.Pairs)
            Assert.Equal(parent.Remote, child.Remote)
            Assert.NotEqual({parent with Seed = 42}, child)
            Assert.Equal("single-gene-mutation", operation)
            Assert.Single changed |> ignore
            Assert.Empty(Genome.validate child)
    let loaded, operation, changed = Evolution.propose 3 true false 7 0 (Some Genome.defaults)
    Assert.True(loaded.Remote)
    Assert.Equal("add-electronics", operation)
    Assert.Equal<string list>(["remote"], changed)

[<Fact>]
let ``wagons have independent passive wheels and movable electronics carriers`` () =
    for pairs in 1..3 do
        for axleHole in 2..4 do
            for battery, receiver in [false,false; false,true; true,false; true,true] do
                let g = {Genome.defaults with Pairs = pairs; Remote = true; TowWagon = true
                                              BatteryOnWagon = battery; ReceiverOnWagon = receiver
                                              WagonAxleHole = axleHole}
                let a = compile Walker g
                Assert.Single a.Drives |> ignore
                Assert.Equal(pairs*2, a.Feet.Length)
                Assert.Equal(2, a.Joints |> List.filter (fun j -> j.Id.StartsWith "wagon-bearing-") |> List.length)
                let hitch = a.Joints |> List.filter (fun j -> j.Id.StartsWith "drawbar-")
                Assert.Equal(2, hitch.Length)
                Assert.All(hitch, fun j ->
                    Assert.Equal("drawbar", j.BodyB)
                    Assert.Equal(Vec3.create 0 0 1, j.Axis)
                    Assert.DoesNotContain(a.Drives, fun drive -> drive.Joint = j.Id))
                let body id = (a.Parts |> List.find (fun p -> p.Id = id)).Body
                Assert.Equal(Some (if battery then "wagon-chassis" else "chassis"), body "payload-battery")
                Assert.Equal(Some (if receiver then "wagon-chassis" else "chassis"), body "payload-receiver")
                Assert.Equal(2, a.Parts |> List.filter (fun p -> p.Part = "56145c01.dat") |> List.length)
                let position id = (a.Parts |> List.find (fun p -> p.Id = id)).Position
                Assert.Equal(battery, (position "payload-battery").X < 0.0)
                Assert.Equal(receiver, (position "payload-receiver").X < 0.0)
                Assert.Equal(float axleHole*20.0, (position "wagon-wheel-0").Y)
                use doc = JsonDocument.Parse(Export.manifest "model.ldr" a)
                Assert.Equal(2, doc.RootElement.GetProperty("groundWheels").GetArrayLength())
                Assert.All(a.Feet, fun f -> Assert.EndsWith("-coupler-part", f.Part))
    let parent = {Genome.defaults with Pairs = 1; Remote = true}
    let child, operation, fields = Evolution.propose 1 true true 42 0 (Some parent)
    Assert.Equal("attach-wagon", operation)
    Assert.True(child.BatteryOnWagon)
    Assert.False(child.ReceiverOnWagon)
    Assert.Contains("towWagon", fields)
    Assert.Contains("batteryOnWagon", fields)

[<Fact>]
let ``invalid and singular genomes are rejected`` () =
    for genome in [
        { Genome.defaults with Pairs = 0 }
        { Genome.defaults with Pairs = 4 }
        { Genome.defaults with CouplerHoles = 0 }
        { Genome.defaults with PhaseQuarterTurns = 4 }
    ] do
        Assert.True((Grammar.compile Walker genome).IsError)
    Assert.True((Kinematics.solve Genome.defaults Double.NaN).IsError)
    Assert.True((Kinematics.solve { Genome.defaults with GroundHoles = 1 } 0).IsError)

[<Fact>]
let ``walker has one motor and a mechanically coupled tripod indexing`` () =
    let a = compile Walker Genome.defaults
    Assert.Single a.Drives |> ignore
    Assert.Equal(6, a.Feet.Length)
    Assert.Equal(10, a.Gears.Length)
    Assert.Equal(29, a.Joints.Length)
    Assert.Equal(24, a.Parts |> List.choose _.Body |> Set.ofList |> Set.count)
    let crank name = a.Parts |> List.find (fun p -> p.Id = name)
    let front = crank "leg-0-0-crank"
    let back = crank "leg-0-1-crank"
    Assert.Equal(-1.0, Vec3.dot front.Rotation.Z back.Rotation.Z, 7)
    let next = crank "leg-1-0-crank"
    Assert.Equal(-1.0, Vec3.dot front.Rotation.Z next.Rotation.Z, 7)
    Assert.Equal("6632.dat", front.Part)
    Assert.Equal(-1.0, Vec3.dot front.Rotation.Y back.Rotation.Y, 7)
    Assert.Contains((Catalog.get front.Part).Ports, fun port -> port.Kind = RoundHole && port.Position.Z = 20.0)
    Assert.All(a.Parts, fun p -> Assert.True(Rotation.valid p.Rotation))

[<Fact>]
let ``axles stop before moving links and bushes retain them inside the frame`` () =
    for pairs in 1..3 do
        let a = compile Walker { Genome.defaults with Pairs = pairs }
        let part id = a.Parts |> List.find (fun p -> p.Id = id)
        for index in 1..(2+(pairs-1)*4) do
            let axle = part $"axle-{index}"
            let lo, hi = Catalog.shaftRange axle.Part
            let isLeg = index >= 2 && (index-2) % 4 = 0
            Assert.Equal((if isLeg then "3707.dat" else "44294.dat"), axle.Part)
            Assert.Equal((if isLeg then -20.0 else -10.0), axle.Position.Z+lo, 7)
            Assert.Equal((if isLeg then 140.0 else 130.0), axle.Position.Z+hi, 7)
            Assert.Equal(60.0, (part $"gear-{index}").Position.Z, 7)
            for z in [20; 100] do
                Assert.Equal(float z, (part $"bush-{index}-{z}").Position.Z, 7)
        let cranks = [for station in 0..pairs-1 -> (part $"leg-{station}-0-crank").Position.X]
        for left, right in List.pairwise cranks do Assert.Equal(240.0, right-left, 7)
        for bridge in a.Parts |> List.filter (fun p -> p.Id.StartsWith "bridge-") do
            Assert.All(cranks, fun x -> Assert.True(abs (bridge.Position.X-x) >= 60.0))

[<Theory>]
[<InlineData(1)>]
[<InlineData(2)>]
[<InlineData(3)>]
let ``all layouts have a connected intended graph`` pairs =
    let a = compile Walker { Genome.defaults with Pairs = pairs }
    let bodies = a.Parts |> List.choose _.Body |> Set.ofList
    let mutable reached = Set.singleton a.Chassis
    while a.Joints |> List.exists (fun j -> reached.Contains j.BodyA <> reached.Contains j.BodyB) do
        for j in a.Joints do
            if reached.Contains j.BodyA || reached.Contains j.BodyB then
                reached <- reached |> Set.add j.BodyA |> Set.add j.BodyB
    Assert.True((bodies = reached))

[<Fact>]
let ``ldraw and manifest are deterministic and culture independent`` () =
    let a = compile Walker Genome.defaults
    let expected = Export.ldraw a
    let culture = CultureInfo.CurrentCulture
    try
        CultureInfo.CurrentCulture <- CultureInfo.GetCultureInfo "da-DK"
        Assert.Equal(expected, Export.ldraw a)
        Assert.Equal(expected, Export.ldraw (compile Walker Genome.defaults))
        use manifest = JsonDocument.Parse(Export.manifest "model.ldr" a)
        Assert.Equal(Export.sha256 expected, manifest.RootElement.GetProperty("ldrawSha256").GetString())
        Assert.Equal(1, manifest.RootElement.GetProperty("schemaVersion").GetInt32())
        Assert.Equal(a.Parts.Length, manifest.RootElement.GetProperty("parts").GetArrayLength())
        for line in expected.Split("\r\n") do
            if line.StartsWith("1 ") then
                Assert.DoesNotContain(",", line)
                Assert.Equal(15, line.Split(' ').Length)
    finally CultureInfo.CurrentCulture <- culture

[<Fact>]
let ``fixtures isolate their intended mechanics`` () =
    let rigid = compile RigidFixture Genome.defaults
    let hinge = compile HingeFixture Genome.defaults
    let motor = compile MotorFixture Genome.defaults
    Assert.Empty rigid.Joints
    Assert.Single hinge.Joints |> ignore
    Assert.Single motor.Drives |> ignore
    Assert.Equal(2, hinge.Parts |> List.choose _.Body |> Set.ofList |> Set.count)

[<Fact>]
let ``foot trajectories close and have nonzero stride and lift`` () =
    let path = Kinematics.footPath Genome.defaults 0
    Assert.Equal(361, path.Points.Length)
    Assert.True(Vec3.distance path.Points.Head (List.last path.Points) < 1e-8)
    Assert.True(path.Stride > 10.0)
    Assert.True(path.Lift > 10.0)

[<Fact>]
let ``sweep is repeatable and exhaustive without duplicate genomes`` () =
    let candidates = Grammar.sweep 42
    Assert.True((candidates = Grammar.sweep 42))
    Assert.False((candidates = Grammar.sweep 43))
    Assert.Equal(32, candidates.Length)
    Assert.Equal(32, Set.ofList candidates |> Set.count)

[<Fact>]
let ``sweep rejects both kinematic failures and overlapping coaxial connectors`` () =
    let mutable accepted = 0
    let mutable rejected = 0
    for genome in Grammar.sweep 42 do
        for pairs in 1..3 do
            let genome = { genome with Pairs = pairs }
            match Kinematics.validate genome, Grammar.compile Walker genome with
            | [], Ok assembly ->
                accepted <- accepted + 1
                Assert.Equal(pairs*2, assembly.Feet.Length)
                Assert.All(assembly.Parts, fun part -> Assert.True(Rotation.valid part.Rotation))
            | expected, Error actual when expected = actual -> rejected <- rejected + 1
            | [], Error errors ->
                rejected <- rejected + 1
                Assert.All(errors, fun error -> Assert.Contains("overlap on the same shaft line", error))
            | _, result -> failwith $"Inconsistent mechanical validation: {result}"
    Assert.Equal(60, accepted)
    Assert.Equal(36, rejected)

[<Fact>]
let ``export refuses to overwrite existing output`` () =
    let path = Path.Combine(Path.GetTempPath(), $"lego-walkers-{Guid.NewGuid():N}")
    try
        let a = compile HingeFixture Genome.defaults
        Export.write path a |> ignore
        Assert.Throws<ArgumentException>(fun () -> Export.write path a |> ignore) |> ignore
        Assert.True(File.Exists(Path.Combine(path, "model.ldr")))
    finally
        for name in ["model.ldr"; "manifest.json"] do
            let file = Path.Combine(path, name)
            if File.Exists file then File.Delete file
        if Directory.Exists path then Directory.Delete path
