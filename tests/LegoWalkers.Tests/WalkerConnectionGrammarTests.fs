module LegoWalkers.WalkerConnectionGrammarTests

open System
open LegoWalkers
open Xunit

let private unwrap result = Result.defaultWith (Realization.json >> failwith) result
let private parent () =
    let program,genome = WalkerSeeds.fourBar 2
    let screened = Realization.screen program 72 |> unwrap
    screened,WalkerRealization.compile screened genome |> unwrap

[<Fact>]
let ``empty conflicts preserve an inherited derivation without broad phase bans`` () =
    let s,r = parent ()
    let result = WalkerConnectionGrammar.derive s r.Genome { Pairs = [] } 4
    let choice = result.Choices |> List.exactlyOne
    Assert.Equal(r.Genome,choice.Construction.Genome)
    Assert.Equal("Assembly -> LayeredLinks JointStacks",choice.Derivation.Head.Production)
    Assert.Equal(1,result.CompleteAssignments)
    Assert.Empty result.CompilerRejections
    Assert.False result.BoundReached

[<Fact>]
let ``grammar jointly assigns layers and connector stacks beyond fixed pin repair`` () =
    let s,r = parent ()
    let body = r.Genome.Bodies |> List.find (fun b -> b.Body = "leg-0-1-coupler")
    let ground = s.Program.Bodies |> List.find _.Grounded
    let fixedPart = (r.Genome.Bodies |> List.find (fun b -> b.Body = ground.Id)).Parts.Head
    let conflicts = { Pairs = [[body.Parts.Head.Id;fixedPart.Id]] }
    let result = WalkerConnectionGrammar.derive s r.Genome conflicts 32
    Assert.NotEmpty result.Choices
    Assert.Equal<WalkerGrammarChoice list>(result.Choices, (WalkerConnectionGrammar.derive s r.Genome conflicts 32).Choices)
    Assert.True(result.VisitedStates <= WalkerConnectionGrammar.maximumStates)
    Assert.True(result.Choices.Length <= 32)
    Assert.Contains(result.Choices,fun choice ->
        let moved = choice.Construction.Genome.Bodies |> List.find (fun b -> b.Body = body.Body)
        abs (moved.Parts.Head.Position.Z-body.Parts.Head.Position.Z) = 20.0
        && choice.Construction.Genome.Connections <> r.Genome.Connections)
    for choice in result.Choices do
        let genome = choice.Construction.Genome
        Assert.Equal(s.MechanismId,genome.MechanismId)
        Assert.Equal<WalkerPlacement list>(r.Genome.Mounts,genome.Mounts)
        Assert.Equal(r.Genome.Speed,genome.Speed)
        Assert.Equal(r.Genome.MaxTorque,genome.MaxTorque)
        Assert.Equal(r.Genome.MotorPart,genome.MotorPart)
        Assert.True((WalkerRealization.compile s genome).IsOk)
        for before,after in List.zip r.Genome.Bodies genome.Bodies do
            if before.Body <> body.Body then Assert.Equal(before,after)
            else
                for a,b in List.zip before.Parts after.Parts do
                    Assert.Equal(a.Position.X,b.Position.X)
                    Assert.Equal(a.Position.Y,b.Position.Y)
                    Assert.Equal(a.Rotation,b.Rotation)
                    Assert.Equal(a.Part,b.Part)
        let moved = genome.Bodies |> List.find (fun b -> b.Body = body.Body)
        let delta = moved.Parts.Head.Position.Z-body.Parts.Head.Position.Z
        for old,foot in List.zip r.Genome.Feet genome.Feet do
            Assert.Equal(old.Depth+(if old.Part = body.Parts.Head.Id then delta else 0.0),foot.Depth)
        Assert.Contains(choice.Derivation,fun step -> step.Production.Contains "RevoluteConnection")

[<Fact>]
let ``grammar validates conflict identities and export bounds`` () =
    let s,r = parent ()
    for limit in [0;33] do
        Assert.Throws<ArgumentException>(fun () -> WalkerConnectionGrammar.derive s r.Genome { Pairs = [] } limit |> ignore) |> ignore
    for pairs in [[[r.Genome.Mounts.Head.Id]];[["missing";r.Genome.Mounts.Head.Id]];
                  [[r.Genome.Mounts.Head.Id;r.Genome.Mounts.Head.Id]]] do
        Assert.Throws<ArgumentException>(fun () -> WalkerConnectionGrammar.derive s r.Genome { Pairs = pairs } 4 |> ignore) |> ignore

[<Fact>]
let ``grammar leaves motor and fixed attachments untouched when no movable production applies`` () =
    let s,r = parent ()
    let ground = s.Program.Bodies |> List.find _.Grounded
    let parts = (r.Genome.Bodies |> List.find (fun b -> b.Body = ground.Id)).Parts
    let result = WalkerConnectionGrammar.derive s r.Genome { Pairs = [[parts[0].Id;parts[1].Id]] } 4
    Assert.Empty result.Choices
    Assert.Equal(1,result.UnchangedConflicts)
    Assert.Empty result.CompilerRejections

[<Fact>]
let ``stock axle grammar can shorten overhang without losing carried parts or changing motion`` () =
    let s,r = parent ()
    Assert.Equal(Some Axle,(Catalog.get "3737.dat").Connector)
    Assert.Equal((-100.0,100.0),Catalog.shaftRange "3737.dat")
    Assert.Contains("3737.dat",Catalog.axleParts)
    let pivot = r.Genome.Connections |> List.find (fun c -> c.Group="bearing-2")
    let longer = {pivot with Placement={pivot.Placement with Part="3708.dat"}}
    let baseline = {r.Genome with Connections=r.Genome.Connections |> List.map (fun c -> if c.Group=pivot.Group then longer else c)}
    let fixedPart = baseline.Bodies |> List.collect _.Parts |> List.find (fun p -> p.Id<>longer.Placement.Id)
    let result = WalkerConnectionGrammar.derive s baseline {Pairs=[[longer.Placement.Id;fixedPart.Id]]} 32
    Assert.NotEmpty result.Choices
    let reached (pin: WalkerPlacement) (part: WalkerPlacement) =
        let lo,hi = Catalog.shaftRange pin.Part
        (Catalog.get part.Part).Ports |> List.exists (fun socket ->
            let point = Vec3.add part.Position (Rotation.apply part.Rotation socket.Position)
            let delta = Vec3.subtract point pin.Position
            let along = Vec3.dot delta pin.Rotation.X
            abs (Vec3.dot (Rotation.apply part.Rotation socket.Axis) pin.Rotation.X)>1.0-1e-6
            && Vec3.length (Vec3.cross delta pin.Rotation.X)<1e-6 && along>=lo-1e-6 && along<=hi+1e-6)
    let required = baseline.Bodies |> List.collect _.Parts |> List.filter (reached longer.Placement)
    Assert.Contains(result.Choices,fun c ->
        let after = c.Construction.Genome.Connections |> List.find (fun p -> p.Group=pivot.Group)
        let lo,hi = Catalog.shaftRange after.Placement.Part
        hi-lo<240.0)
    for choice in result.Choices do
        let after = choice.Construction.Genome.Connections |> List.find (fun p -> p.Group=pivot.Group)
        for part in required do Assert.True(reached after.Placement part)
        Assert.Equal(s.MechanismId,choice.Construction.Genome.MechanismId)
        Assert.Equal(baseline.Speed,choice.Construction.Genome.Speed)
        Assert.Equal(baseline.MaxTorque,choice.Construction.Genome.MaxTorque)
        Assert.Equal<WalkerPlacement list>(baseline.Mounts,choice.Construction.Genome.Mounts)
        Assert.True((WalkerRealization.compile s choice.Construction.Genome).IsOk)
