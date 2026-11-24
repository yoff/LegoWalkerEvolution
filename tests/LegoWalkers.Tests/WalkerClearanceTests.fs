module LegoWalkers.WalkerClearanceTests

open System
open System.IO
open System.Text.Json.Nodes
open LegoWalkers
open Xunit

let private point x y z = Vec3.create x y z
let private unwrap result = Result.defaultWith (Realization.json >> failwith) result
let private parent () =
    let program,genome = WalkerSeeds.fourBar 2
    let screened = Realization.screen program 72 |> unwrap
    screened,WalkerRealization.compile screened genome |> unwrap

[<Fact>]
let ``segment distance handles crossing parallel depth and degenerate rods`` () =
    let distance,_,_ = WalkerClearance.segmentDistance (point -10 0 0) (point 10 0 0) (point 0 -10 0) (point 0 10 0)
    Assert.Equal(0.0,distance,10)
    let distance,_,_ = WalkerClearance.segmentDistance (point -10 0 0) (point 10 0 0) (point -10 0 20) (point 10 0 20)
    Assert.Equal(20.0,distance,10)
    let distance,_,_ = WalkerClearance.segmentDistance (point 0 0 0) (point 0 0 0) (point 10 0 0) (point 20 0 0)
    Assert.Equal(10.0,distance,10)

[<Fact>]
let ``segment box distance checks segment interiors and outside corners`` () =
    let lo,hi = point -5 -5 -5,point 5 5 5
    let distance,_,_ = WalkerClearance.segmentBoxDistance (point -100 0 0) (point 100 0 0) lo hi
    Assert.Equal(0.0,distance,10)
    let distance,_,_ = WalkerClearance.segmentBoxDistance (point -100 8 9) (point 100 8 9) lo hi
    Assert.Equal(5.0,distance,10)
    let distance,_,_ = WalkerClearance.segmentBoxDistance (point 8 9 5) (point 8 9 5) lo hi
    Assert.Equal(5.0,distance,10)

let private synthetic depth =
    let s,r = parent ()
    let bodies = s.Program.Bodies |> List.filter (fun b -> not b.Grounded) |> List.take 2
    let first,second = bodies[0].Id,bodies[1].Id
    let beam id x z =
        { Id = id; Part = "32316.dat"; Color = 4; Position = point x 0 z
          Rotation = Rotation.beamWithNormal (point 0 0 1) (point 1 0 0) }
    let frame = { s.Frames.Head with Poses = s.Frames.Head.Poses |> Map.map (fun _ _ -> { Origin = point 0 0 0; Angle = 0.0 }) }
    let screened = { s with Frames = [frame;{ frame with InputAngle = 2.0*Math.PI }] }
    let genome = { r.Genome with
                       Bodies = [{ Body = first; Parts = [beam "a" 0 0] };{ Body = second; Parts = [beam "b" 0 depth] }]
                       Connections = []; Mounts = [] }
    screened,genome

[<Fact>]
let ``coincident motion-plane links only interfere when physical depth overlaps`` () =
    let s,g = synthetic 20.0
    let report = WalkerClearance.evaluateFrames s s.Frames g 0
    Assert.Equal(0,report.SuspectPairs)
    let s,g = synthetic 0.0
    let report = WalkerClearance.evaluateFrames s s.Frames g 0
    Assert.Equal(1,report.SuspectPairs)
    Assert.Equal(-6.0,report.Contacts.Head.ClearanceLdu,10)
    Assert.Equal("warning",report.Status)
    Assert.Equal(0,report.DynamicTrials)

[<Fact>]
let ``connected links are still checked beyond the intended joint region`` () =
    let s,g = synthetic 0.0
    let first,second = g.Bodies[0].Body,g.Bodies[1].Body
    let program = { s.Program with
                       Hinges = { Id = "test-hinge"; A = Motion.port first "test"; B = Motion.port second "test" }::s.Program.Hinges
                       Bodies = s.Program.Bodies |> List.map (fun body ->
                           { body with Sites = Motion.site "test" 0 0::body.Sites }) }
    let graph = Motion.compile program |> Result.defaultWith (String.concat "; " >> failwith)
    let s = { s with Program = program; Graph = graph }
    let pin = { Id = "pivot-test"; Part = "32002.dat"; Color = 8; Position = point 0 0 0
                Rotation = { X = point 0 0 1; Y = point 0 1 0; Z = point -1 0 0 } }
    let g = { g with Connections = [{ Group = "test"; Hinges = ["test-hinge"]; Placement = pin }] }
    let report = WalkerClearance.evaluateFrames s s.Frames g 0
    Assert.Equal(1,report.SuspectPairs)
    Assert.True(abs report.Contacts.Head.PointA.X >= 12.0-1e-6)

[<Fact>]
let ``diagnostic sampling and construction alternatives are bounded and retain the original`` () =
    let s,r = parent ()
    let frames,report = WalkerClearance.inspect s r.Genome
    Assert.InRange(report.RefinedIntervals,0,8)
    Assert.InRange(frames.Length,73,97)
    Assert.True(report.EndpointRepeats)
    Assert.Equal(1.0,report.MotorTurnsCovered,8)
    let alternatives = WalkerConstruction.alternatives s r.Genome 2
    Assert.InRange(alternatives.Length,1,2)
    Assert.Equal(r.Genome,alternatives.Head.Genome)
    Assert.Equal<WalkerConstructionChoice list>(alternatives,WalkerConstruction.alternatives s r.Genome 2)
    for choice in alternatives do
        WalkerRealization.compile s choice.Genome |> unwrap |> ignore
        Assert.Equal(r.Genome.MechanismId,choice.Genome.MechanismId)
        Assert.Equal(r.Genome.Speed,choice.Genome.Speed)
        Assert.Equal(r.Genome.MaxTorque,choice.Genome.MaxTorque)

[<Fact>]
let ``reduction diagnostics cover the complete five-turn gait not just one motor revolution`` () =
    let s,r = parent ()
    let edit = WalkerEvolution.physicalMutations s r.Genome |> List.find (fun edit -> edit.Operation = "gear-reduction")
    let child = Realization.screen edit.Program 72 |> unwrap
    let g = { edit.Physical.Value with MechanismId = child.MechanismId }
    WalkerRealization.compile child g |> unwrap |> ignore
    let frames,report = WalkerClearance.inspect child g
    Assert.Equal(5.0,report.MotorTurnsCovered,8)
    Assert.True(report.EndpointRepeats)
    Assert.InRange(frames.Length,361,385)

[<Fact>]
let ``motion feedback never fabricates a repair for an unmodeled mount`` () =
    let s,r = parent ()
    let feedback = { Kind = "motion-clearance"; Parts = [r.Genome.Mounts.Head.Id]; Bodies = [] }
    Assert.Empty(WalkerConstruction.repairs s r.Genome feedback 2)
    Assert.Throws<ArgumentException>(fun () -> WalkerConstruction.repairs s r.Genome feedback 33 |> ignore) |> ignore

[<Fact>]
let ``hardware construction changes are identified rather than mislabeled as inherited`` () =
    let s,r = parent ()
    let edit = WalkerLayout.receiverEdits s r.Genome |> List.head
    let choice = WalkerConstruction.describe s r.Genome edit.Genome
    Assert.Equal("motion-clearance-layout",choice.Motif)
    Assert.NotEmpty(choice.AffectedBodies)
    Assert.Equal(edit.Genome,choice.Genome)

[<Fact>]
let ``prescribed phase exports identify every physical part including connectors and mounts`` () =
    let s,r = parent ()
    let frames,report = WalkerClearance.inspect s r.Genome
    let directory = Path.Combine(Path.GetTempPath(),"walker-phases-"+Guid.NewGuid().ToString("N"))
    Directory.CreateDirectory directory |> ignore
    try
        WalkerClearance.writePhases directory s r frames report
        let document = JsonNode.Parse(File.ReadAllText(Path.Combine(directory,"motion-phases.json")))
        let ids = document["parts"].AsArray() |> Seq.map (fun p -> p.GetValue<string>()) |> Set.ofSeq
        Assert.Equal(r.Assembly.Parts.Length,ids.Count)
        Assert.All(r.Genome.Connections,fun c -> Assert.Contains(c.Placement.Id,ids))
        Assert.All(r.Genome.Mounts,fun p -> Assert.Contains(p.Id,ids))
        Assert.Equal(frames.Length,document["frames"].AsArray().Count)
        Assert.True(document["endpointRepeats"].GetValue<bool>())
        Assert.Equal(r.GenotypeId,document["genotypeId"].GetValue<string>())
        Assert.All(document["frames"].AsArray(),fun frame ->
            Assert.Equal(s.Program.Bodies.Length,frame["bodies"].AsArray().Count))
    finally
        Directory.Delete(directory,true)

[<Fact>]
let ``measured receiver conflicts offer layout repairs independently of approximate contact targets`` () =
    let s,r = parent ()
    let choices = WalkerConstruction.measuredLayouts s r.Genome ["payload-receiver"] 1
    Assert.NotEmpty choices
    for choice in choices do
        WalkerRealization.compile s choice.Genome |> unwrap |> ignore
        Assert.Equal("motion-clearance-receiver-layout",choice.Motif)
        Assert.NotEqual<WalkerGenome>(r.Genome,choice.Genome)
        Assert.Equal(r.Genome.Speed,choice.Genome.Speed)
        Assert.Equal(r.Genome.MaxTorque,choice.Genome.MaxTorque)
        Assert.Equal<WalkerFootChoice list>(r.Genome.Feet,choice.Genome.Feet)

[<Fact>]
let ``measured mounting pin conflicts target the supported receiver layout that carries them`` () =
    let s,r = parent ()
    let edit = WalkerLayout.receiverEdits s r.Genome |> List.head
    let pin = r.Genome.Mounts |> List.find (fun p ->
        edit.Genome.Mounts |> List.exists (fun moved -> moved.Id = p.Id && moved <> p))
    let choices = WalkerConstruction.measuredLayouts s r.Genome [pin.Id] 1
    Assert.NotEmpty choices
    for choice in choices do
        Assert.Equal("motion-clearance-receiver-layout",choice.Motif)
        let moved = choice.Genome.Mounts |> List.find (fun p -> p.Id = pin.Id)
        Assert.NotEqual<WalkerPlacement>(pin,moved)
        Assert.True((WalkerRealization.compile s choice.Genome).IsOk)
