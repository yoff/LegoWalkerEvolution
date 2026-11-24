module LegoWalkers.WalkerEvolutionTests

open System
open System.IO
open LegoWalkers
open Xunit

let private unwrap result = Result.defaultWith (Realization.json >> failwith) result

let private assertOutwardLevelL (s: ScreenedMotion) (g: WalkerGenome) =
    for foot in s.Program.Feet |> List.filter (fun f -> f.Port.Body.EndsWith("-sole-output")) do
        let prefix = foot.Port.Body.Substring(0,foot.Port.Body.Length-"-sole-output".Length)
        let l = g.Bodies |> List.find (fun b -> b.Body=prefix+"-upper-output") |> _.Parts |> List.exactlyOne
        let world p = Vec3.add l.Position (Rotation.apply l.Rotation p) |> fun p -> {p with Z=0.0}
        let position frame suffix site =
            Motion.position s.Graph frame (Motion.port (prefix+suffix) site) |> Result.defaultWith failwith
            |> fun p -> {p with Z=0.0}
        let corner = position s.Frames.Head "-upper-output" "corner"
        let toe = position s.Frames.Head "-upper-output" "toe"
        Assert.InRange(Vec3.distance corner (world (Vec3.create 0 0 80)),0.0,1e-6)
        Assert.InRange(Vec3.distance toe (world (Vec3.create 40 0 80)),0.0,1e-6)
        let top = world Vec3.zero
        Assert.InRange(abs(top.X-corner.X),0.0,1e-6)
        Assert.True(top.Y<corner.Y-60.0)
        for frame in s.Frames do
            let corner = position frame "-upper-output" "corner"
            let toe = position frame "-upper-output" "toe"
            let anchor = position frame "-leg-a" "root"
            Assert.True(Vec3.dot (Vec3.subtract toe corner) (Vec3.subtract corner anchor)>1e-6)
let private parent feet =
    let p, g = WalkerSeeds.fourBar feet
    let s = Realization.screen p 72 |> unwrap
    s, WalkerRealization.compile s g |> unwrap

let private supportsPin (pin: WalkerPlacement) (part: WalkerPlacement) =
    let lo,hi = Catalog.shaftRange pin.Part
    (Catalog.get part.Part).Ports |> List.exists (fun port ->
        let delta = Vec3.subtract (Vec3.add part.Position (Rotation.apply part.Rotation port.Position)) pin.Position
        let along = Vec3.dot delta pin.Rotation.X
        port.Kind = RoundHole
        && abs (Vec3.dot (Rotation.apply part.Rotation port.Axis) pin.Rotation.X) > 1.0-1e-6
        && Vec3.length (Vec3.cross delta pin.Rotation.X) < 1e-6
        && along >= lo-1e-6 && along <= hi+1e-6)

// Keep existing realization and diagram witnesses independent of the sampling policy.
let private parameterWitness s genome =
    let random = Random 42
    WalkerEvolution.mutations s genome
    |> List.map (fun mutation -> random.Next(), mutation)
    |> List.sortBy fst
    |> List.head
    |> snd

let private physicalWitness operation s genome =
    let attempts =
        WalkerEvolution.physicalMutations s genome
        |> List.filter (fun m -> m.Operation = operation)
        |> List.map (fun m ->
            match Realization.screen m.Program 72 with
            | Error e -> Error (Realization.json e)
            | Ok child ->
                let concrete = WalkerEvolution.realize s genome m child 1 |> unwrap |> List.exactlyOne
                WalkerRealization.compile child concrete |> Result.map (fun robot -> m,child,robot) |> Result.mapError Realization.json)
    let success = attempts |> List.choose (function Ok result -> Some result | _ -> None)
    let errors = attempts |> List.choose (function Error error -> Some error | _ -> None) |> String.concat "\n"
    Assert.True(not success.IsEmpty, $"{operation}: {errors}")
    success.Head

[<Fact>]
let ``motor direction reverses persistent control without changing walking geometry or torque`` () =
    let s,r = parent 2
    let mutation = WalkerEvolution.physicalMutations s r.Genome
                   |> List.filter (fun m -> m.Operation="motor-direction") |> List.exactlyOne
    Assert.Equal(s.Program,mutation.Program)
    Assert.Equal("genome/speed",mutation.Changes.Head.Path)
    let concrete = WalkerEvolution.realize s r.Genome mutation s 1 |> unwrap |> List.exactlyOne
    Assert.Equal({r.Genome with Speed = -r.Genome.Speed},concrete)
    let robot = WalkerRealization.compile s concrete |> unwrap
    Assert.Equal(-r.Genome.Speed,robot.Assembly.Drives.Head.Speed)
    Assert.Equal(r.Genome.MaxTorque,robot.Assembly.Drives.Head.MaxTorque)
    Assert.True(r.Assembly.Parts=robot.Assembly.Parts)
    Assert.NotEqual<string>(r.GenotypeId,robot.GenotypeId)
    let reverse = WalkerEvolution.physicalMutations s concrete |> List.find (fun m -> m.Operation="motor-direction")
    Assert.Equal(r.Genome,reverse.Physical.Value)

[<Fact>]
let ``gear reduction reuses an existing compound shaft without moving downstream gears`` () =
    let s,r = parent 2
    let edge = s.Program.Transmissions |> List.find (fun t -> t.Input=s.Program.Motor.Body)
    let output = r.Genome.Bodies |> List.find (fun b -> b.Body=edge.Output)
    let gear = output.Parts |> List.find (fun p -> (Catalog.gearTeeth p.Part).IsSome)
    let low = {gear with Id=gear.Id+"-existing-compound";Position={gear.Position with Z=gear.Position.Z-60.0}}
    let bodies = r.Genome.Bodies |> List.map (fun b ->
        if b.Body=output.Body then {b with Parts=b.Parts @ [low]} else b)
    let genome = {r.Genome with Bodies=bodies}
    let edit = WalkerStructural.gearingEdits s genome |> List.exactlyOne
    Assert.Equal(-0.2,(edit.Program.Transmissions |> List.find (fun t -> t.Input=edge.Input)).Ratio)
    Assert.Equal(genome.Bodies |> List.sumBy (fun b -> b.Parts.Length),
                 edit.Genome.Bodies |> List.sumBy (fun b -> b.Parts.Length))
    Assert.Equal(genome.Speed,edit.Genome.Speed)
    Assert.Equal(genome.MaxTorque,edit.Genome.MaxTorque)
    for body in genome.Bodies do
        let changed = edit.Genome.Bodies |> List.find (fun b -> b.Body=body.Body)
        for before in body.Parts do
            let after = changed.Parts |> List.find (fun p -> p.Id=before.Id)
            Assert.Equal(before.Position,after.Position)
            Assert.Equal(before.Rotation,after.Rotation)
    Assert.Contains((edit.Genome.Bodies |> List.find (fun b -> b.Body=output.Body)).Parts,fun p -> p=low)

[<Theory>]
[<InlineData("lengthen-foot")>]
[<InlineData("bend-foot")>]
[<InlineData("gear-reduction")>]
[<InlineData("duplicate-branch")>]
let ``physical operators have actual catalog realizations`` operation =
    let s,r = parent 2
    let mutation,child,robot = physicalWitness operation s r.Genome
    Assert.NotEqual<string>(r.GenotypeId, robot.GenotypeId)
    Assert.Equal(mutation, WalkerEvolution.physicalMutations s r.Genome |> List.find ((=) mutation))
    if operation = "gear-reduction" then
        Assert.Contains(robot.Assembly.Gears, fun gear -> abs (gear.Ratio-0.2) < 1e-6)
        Assert.Contains(robot.Assembly.Parts, fun part -> part.Part = "3647.dat")
        Assert.Contains(robot.Assembly.Parts, fun part -> part.Part = "3649.dat")
        Assert.Equal(4, robot.Assembly.Feet.Length)
    elif operation = "duplicate-branch" then Assert.Equal(6, robot.Assembly.Feet.Length)
    else
        Assert.NotEqual<Result<Vec3,string> list>(s.Program.Feet |> List.map (fun f -> Motion.position s.Graph s.Frames.Head f.Port),
                        child.Program.Feet |> List.map (fun f -> Motion.position child.Graph child.Frames.Head f.Port))

[<Fact>]
let ``Walker1 grows two additional generic branches and can lose them`` () =
    let p,g = Walker1Seeds.walker 2
    let s = Realization.screen p 72 |> unwrap
    let r = WalkerRealization.compile s g |> unwrap
    let _,three,r3 = physicalWitness "duplicate-branch" s r.Genome
    let _,four,r4 = physicalWitness "duplicate-branch" three r3.Genome
    Assert.Equal(4, r4.Genome.Feet.Length)
    Assert.Equal(32, four.Program.Bodies.Length)
    let _,smaller,rSmall = physicalWitness "remove-branch" four r4.Genome
    Assert.Equal(3, smaller.Program.Feet.Length)
    let _,two,rTwo = physicalWitness "remove-branch" smaller rSmall.Genome
    Assert.Equal(2, two.Program.Feet.Length)
    Assert.DoesNotContain(WalkerEvolution.physicalMutations two rTwo.Genome, fun m -> m.Operation = "remove-branch")

[<Fact>]
let ``native double parallelogram replacement preserves actuators and yields level soles`` () =
    let program,genome = Walker1Seeds.walker 2
    let s = Realization.screen program 72 |> unwrap
    let original = WalkerRealization.compile s genome |> unwrap
    let edits = WalkerLevelFeet.edits s original.Genome
    Assert.NotEmpty edits
    Assert.InRange(edits.Length,1,4*program.Feet.Length)
    Assert.Contains(edits,fun edit -> edit.Changes |> List.exists (fun (path,_,after) ->
        path.EndsWith("/l-layout-preference") && after=[0.0]))
    Assert.Equal<WalkerPhysicalEdit list>(edits,WalkerLevelFeet.edits s original.Genome)
    for edit in edits |> List.groupBy (fun e -> e.Changes.Head |> fun (path,_,_) -> path) |> List.map (fun (_,values) -> values.Head) do
        let child = Realization.screen edit.Program 72 |> unwrap
        let robot = WalkerRealization.compile child edit.Genome |> unwrap
        assertOutwardLevelL child robot.Genome
        Assert.Equal(program.Bodies.Length, child.Program.Bodies.Length)
        Assert.Equal(program.Motor, child.Program.Motor)
        Assert.Equal<MotionTransmission list>(program.Transmissions, child.Program.Transmissions)
        Assert.Equal(original.Genome.Speed, robot.Genome.Speed)
        Assert.Equal(original.Genome.MaxTorque, robot.Genome.MaxTorque)
        Assert.Equal(original.Genome.MotorPart, robot.Genome.MotorPart)
        Assert.Equal<string list>(program.Feet |> List.map _.Id, child.Program.Feet |> List.map _.Id)
        let transferred = child.Program.Feet |> List.find (fun foot -> foot.Port.Body.StartsWith("level-foot-"))
        Assert.All(child.Frames, fun frame ->
            Assert.InRange(abs frame.Poses[transferred.Port.Body].Angle, 0.0, 1e-6))
        Assert.Contains(robot.Assembly.Parts, fun p -> p.Part="32526.dat")
        for body in robot.Genome.Bodies do
            if program.Transmissions |> List.exists (fun t -> t.Input=body.Body || t.Output=body.Body) then
                for part in body.Parts do
                    for socket in (Catalog.get part.Part).Ports |> List.filter (fun p -> p.Kind=AxleHole) do
                        let point = Vec3.add part.Position (Rotation.apply part.Rotation socket.Position)
                        Assert.Contains(robot.Genome.Connections, fun c ->
                            if (Catalog.get c.Placement.Part).Connector<>Some Axle then false else
                                let pin = c.Placement
                                let delta = Vec3.subtract point pin.Position
                                let lo,hi = Catalog.shaftRange pin.Part
                                let along = Vec3.dot delta pin.Rotation.X
                                Vec3.length(Vec3.cross delta pin.Rotation.X)<1e-6
                                && along-10.0>=lo-1e-6 && along+10.0<=hi+1e-6)
        for body in original.Genome.Bodies do
            match robot.Genome.Bodies |> List.tryFind (fun b -> b.Body=body.Body) with
            | Some retained ->
                Assert.All(body.Parts, fun part ->
                    let actual = retained.Parts |> List.find (fun p -> p.Id=part.Id)
                    if actual<>part then
                        if actual.Part=part.Part then
                            Assert.Contains(edit.Changes, fun (path,_,_) -> path = $"parts/{part.Id}/depth")
                            Assert.Equal(part,{actual with Position=part.Position})
                            Assert.InRange(abs(actual.Position.Z-part.Position.Z),9.999999,10.000001)
                        else
                            Assert.Contains(edit.Changes, fun (path,_,_) -> path = $"parts/{part.Id}/upright:{part.Part}->{actual.Part}")
                            for socket in (Catalog.get part.Part).Ports do
                                let point = Vec3.add part.Position (Rotation.apply part.Rotation socket.Position)
                                Assert.Contains((Catalog.get actual.Part).Ports, fun fitted ->
                                    Vec3.distance point (Vec3.add actual.Position (Rotation.apply actual.Rotation fitted.Position))<1e-6)
                    else Assert.Contains(part,retained.Parts))
            | None -> ()
        for mount in original.Genome.Mounts do Assert.Contains(mount,robot.Genome.Mounts)
        let next = WalkerLevelFeet.edits child robot.Genome
        Assert.NotEmpty next
        let both = Realization.screen next.Head.Program 72 |> unwrap
        assertOutwardLevelL both next.Head.Genome
        Assert.All(both.Program.Feet, fun foot ->
            Assert.All(both.Frames, fun frame ->
                Assert.InRange(abs frame.Poses[foot.Port.Body].Angle,0.0,1e-6)))

[<Fact>]
let ``native plate foot defaults upgrade a generated replacement without changing control mutations`` () =
    let program,source = Walker1Seeds.walker 2
    let s = Realization.screen program 72 |> unwrap
    let original = WalkerRealization.compile s source |> unwrap
    let edit = WalkerLevelFeet.edits s original.Genome |> List.head
    let mutation: WalkerMutation = {
        Program=edit.Program;Operation=edit.Operation
        Changes=edit.Changes |> List.map (fun (path,before,after) -> {Path=path;Before=before;After=after})
        RotatedPart=None;Dyad=None;RemovedBodies=[];FootReplacement=None;Physical=Some edit.Genome
        Components=[];FamilyUpdate=None
    }
    let child = Realization.screen mutation.Program 72 |> unwrap
    let concrete = WalkerEvolution.realize s original.Genome mutation child 1 |> unwrap |> List.exactlyOne
    let robot = WalkerRealization.compile child concrete |> unwrap
    let parts = robot.Genome.Bodies |> List.collect _.Parts |> List.map (fun part -> part.Id,part) |> Map.ofList
    Assert.Equal(original.Genome.Speed,robot.Genome.Speed)
    Assert.Equal(program.Motor,child.Program.Motor)
    Assert.Equal<MotionTransmission list>(program.Transmissions,child.Program.Transmissions)
    for foot in robot.Genome.Feet do
        let contacts = WalkerFootChoice.contacts foot
        Assert.Equal(2,contacts.Length)
        Assert.All(contacts,fun contact -> Assert.Equal("3710.dat",parts[contact.Part].Part))
    let prepared,_ = WalkerFootPlates.prepare child robot.Genome |> unwrap
    Assert.Equal(robot.Genome,prepared)
    let reverse = {mutation with Program=child.Program;Operation="motor-direction"
                                 Physical=Some {robot.Genome with Speed = -robot.Genome.Speed}
                                 Changes=[{Path="genome/speed";Before=[robot.Genome.Speed];After=[-robot.Genome.Speed]}]}
    let reversed = WalkerEvolution.realize child robot.Genome reverse child 1 |> unwrap |> List.exactlyOne
    Assert.Equal({robot.Genome with Speed = -robot.Genome.Speed},reversed)

[<Fact>]
let ``shared leg templates retain their sharing through default plate foot preparation`` () =
    let program,source = Walker1Seeds.walker 2
    let s = Realization.screen program 72 |> unwrap
    let shared = WalkerLegFamilies.adopt "walker-legs" (program.Feet |> List.map _.Id) program source
    let prepared,_ = WalkerFootPlates.prepare s shared |> unwrap
    let robot = WalkerRealization.compile s prepared |> unwrap
    Assert.Equal(shared.LegFamilies,robot.Genome.LegFamilies)
    Assert.All(robot.Genome.Feet,fun foot -> Assert.Equal(2,(WalkerFootChoice.contacts foot).Length))
    let repeated,_ = WalkerFootPlates.prepare s robot.Genome |> unwrap
    Assert.Equal(robot.Genome,repeated)

[<Fact>]
let ``flat native legs with inward L remain replaceable without duplicating ground anchors`` () =
    let program,genome = Walker1Seeds.walker 2
    let s = Realization.screen program 72 |> unwrap
    let edit = WalkerLevelFeet.edits s genome |> List.head
    let foot = edit.Program.Feet |> List.find (fun f -> f.Port.Body.EndsWith("-sole-output"))
    let prefix = foot.Port.Body.Substring(0,foot.Port.Body.Length-"-sole-output".Length)
    let upper = edit.Program.Bodies |> List.find (fun b -> b.Id=prefix+"-upper-output")
    let toe = upper.Sites |> List.find (fun p -> p.Id="toe") |> _.Position.X
    let shift delta (p: Vec3) = {p with X=p.X+delta}
    let badBodies = edit.Program.Bodies |> List.map (fun b ->
        if b.Id=upper.Id then
            {b with Sites=b.Sites |> List.map (fun p -> if p.Id="toe" then {p with Position=shift (-2.0*toe) p.Position} else p)}
        elif b.Id=prefix+"-sole-parallel" then {b with Initial={b.Initial with Origin=shift (-2.0*toe) b.Initial.Origin}}
        elif b.Id=foot.Port.Body then
            {b with Sites=b.Sites |> List.map (fun p -> {p with Position={p.Position with X= -p.Position.X}})}
        else b)
    let badProgram = {edit.Program with Bodies=badBodies}
    let badGenome = {
        edit.Genome with MechanismId=Realization.mechanismId badProgram
                         Bodies=edit.Genome.Bodies |> List.map (fun b ->
                             if b.Body=upper.Id then
                                 {b with Parts=b.Parts |> List.map (fun p ->
                                     {p with Rotation={p.Rotation with X=Vec3.scale -1.0 p.Rotation.X;Y=Vec3.scale -1.0 p.Rotation.Y}})}
                             elif b.Body=prefix+"-sole-parallel" || b.Body=foot.Port.Body then
                                 let delta = if b.Body=foot.Port.Body then -toe else -2.0*toe
                                 {b with Parts=b.Parts |> List.map (fun p -> {p with Position=shift delta p.Position})}
                             else b)
                         Connections=edit.Genome.Connections |> List.map (fun c ->
                             if c.Group=prefix+"-sole-base" || c.Group=prefix+"-sole-tip" then
                                 {c with Placement={c.Placement with Position=shift (-2.0*toe) c.Placement.Position}}
                             else c)
    }
    let bad = Realization.screen badProgram 72 |> unwrap
    WalkerRealization.compile bad badGenome |> unwrap |> ignore
    Assert.All(bad.Frames,fun frame -> Assert.InRange(abs frame.Poses[foot.Port.Body].Angle,0.0,1e-6))
    let repaired = WalkerLevelFeet.edits bad badGenome
                   |> List.find (fun e -> e.Changes |> List.exists (fun (path,_,_) -> path=$"feet/{foot.Id}/level-loop"))
    let catalog = WalkerEvolution.editsWithPolicy WalkerBuilder.defaultPolicy bad badGenome
                  |> List.filter (fun (_,_,m) -> m.Changes |> List.exists (fun c -> c.Path=$"feet/{foot.Id}/level-loop"))
    Assert.NotEmpty catalog
    let _,_,first = catalog.Head
    Assert.Equal<WalkerGenome option>(Some repaired.Genome,first.Physical)
    Assert.Equal(repaired.Program,first.Program)
    let child = Realization.screen repaired.Program 72 |> unwrap
    WalkerRealization.compile child repaired.Genome |> unwrap |> ignore
    assertOutwardLevelL child repaired.Genome
    let replaced = child.Program.Feet |> List.find (fun f -> f.Id=foot.Id)
    let newPrefix = replaced.Port.Body.Substring(0,replaced.Port.Body.Length-"-sole-output".Length)
    let before = Motion.position bad.Graph bad.Frames.Head (Motion.port (prefix+"-leg-a") "root") |> Result.defaultWith failwith
    let after = Motion.position child.Graph child.Frames.Head (Motion.port (newPrefix+"-leg-a") "root") |> Result.defaultWith failwith
    Assert.InRange(Vec3.distance before after,0.0,1e-6)
    for before,after in List.zip bad.Frames child.Frames do
        let oldKnee = Motion.position bad.Graph before (Motion.port (prefix+"-leg-a") "tip") |> Result.defaultWith failwith
        let newKnee = Motion.position child.Graph after (Motion.port (newPrefix+"-leg-a") "tip") |> Result.defaultWith failwith
        Assert.InRange(Vec3.distance oldKnee newKnee,0.0,1e-6)
    for body in child.Program.Bodies do
        Assert.Equal(body.Sites.Length,body.Sites |> List.map _.Id |> List.distinct |> List.length)
    Assert.Equal(bad.Program.Motor,child.Program.Motor)
    Assert.Equal<MotionTransmission list>(bad.Program.Transmissions,child.Program.Transmissions)
    Assert.Equal(badGenome.Speed,repaired.Genome.Speed)
    Assert.Equal(badGenome.MaxTorque,repaired.Genome.MaxTorque)

[<Theory>]
[<InlineData(2)>]
[<InlineData(3)>]
let ``hinged foot mutations preserve whole walker hardware and produce orientation preserving soles`` feet =
    let s,r = parent feet
    let edits = WalkerLoops.edits s r.Genome
    Assert.NotEmpty edits
    Assert.Equal<WalkerPhysicalEdit list>(edits, WalkerLoops.edits s r.Genome)
    let sides = edits |> List.map (fun e ->
        let added = e.Genome.Bodies |> List.find (fun b -> b.Body.EndsWith "-output")
        sign added.Parts.Head.Position.Z) |> Set.ofList
    Assert.Contains(-1, sides)
    Assert.Contains(1, sides)
    for edit in edits do
        let child = Realization.screen edit.Program 72 |> unwrap
        let offspring = WalkerRealization.compile child edit.Genome |> unwrap
        Assert.Equal(s.Program.Bodies.Length+2, child.Program.Bodies.Length)
        Assert.Equal(s.Program.Hinges.Length+3, child.Program.Hinges.Length)
        Assert.Equal(s.Program.Motor, child.Program.Motor)
        Assert.Equal<MotionTransmission list>(s.Program.Transmissions, child.Program.Transmissions)
        Assert.Equal<string list>(s.Program.Feet |> List.map _.Id, child.Program.Feet |> List.map _.Id)
        Assert.Equal(r.Genome.Speed, offspring.Genome.Speed)
        Assert.Equal(r.Genome.MaxTorque, offspring.Genome.MaxTorque)
        Assert.Equal(r.Genome.MotorPart, offspring.Genome.MotorPart)
        Assert.Equal<WalkerPlacement list>(r.Genome.Mounts, offspring.Genome.Mounts)
        for body in r.Genome.Bodies do
            Assert.Equal<WalkerPlacement list>(body.Parts, (offspring.Genome.Bodies |> List.find (fun b -> b.Body=body.Body)).Parts)
        let sole = child.Program.Bodies |> List.find (fun b -> b.Id.EndsWith "-output")
        Assert.All(child.Frames, fun frame -> Assert.InRange(abs frame.Poses[sole.Id].Angle, 0.0, 1e-6))
        Assert.Equal(2*feet+2, offspring.Assembly.Feet.Length)
        Assert.Contains(offspring.Assembly.Parts, fun part -> part.Part="3701.dat" && part.Body=Some sole.Id)
        Assert.Equal(3, offspring.Groups |> List.filter (fun (_,hs,_) ->
            hs |> List.exists (fun id -> id.StartsWith "hinged-foot-")) |> List.length)

[<Fact>]
let ``hinged foot mutations do not extend an already shared corner`` () =
    let s,r = parent 3
    let edit = WalkerLoops.edits s r.Genome |> List.head
    let child = Realization.screen edit.Program 72 |> unwrap
    let transferred = child.Program.Feet |> List.find (fun foot -> foot.Port.Body.EndsWith "-output")
    let next = WalkerLoops.edits child edit.Genome
    Assert.NotEmpty next
    for mutation in next do
        Assert.Equal(transferred, mutation.Program.Feet |> List.find (fun foot -> foot.Id=transferred.Id))

[<Fact>]
let ``hinged foot mutations respect the complete walker body limit`` () =
    let program,genome = Walker1Seeds.walker 2
    let screened = Realization.screen program 72 |> unwrap
    let robot = WalkerRealization.compile screened genome |> unwrap
    let _,three,r3 = physicalWitness "duplicate-branch" screened robot.Genome
    let _,four,r4 = physicalWitness "duplicate-branch" three r3.Genome
    Assert.Equal(32, four.Program.Bodies.Length)
    Assert.Empty(WalkerLoops.edits four r4.Genome)

[<Fact>]
let ``mutation sampling gives each applicable family equal first proposal exposure`` () =
    let s,r = parent 4
    let sample = WalkerEvolution.chooseWithSelection s r.Genome
    let selections = [for seed in 0..4095 -> sample seed 0]
    let families = (snd selections.Head).availableFamilies
    Assert.Contains(families, fun family -> family.operation = "add-hinged-foot")
    Assert.Contains(families, fun family -> family.operation = "battery-layout")
    Assert.Contains(families, fun family -> family.operation = "receiver-layout")
    Assert.Contains(families, fun family -> family.operation = "motor-layout")
    Assert.Contains(families, fun family -> family.operation = "limb-length")
    Assert.True((families |> List.maxBy _.edits).edits > 2*(families |> List.minBy _.edits).edits)
    let counts = selections |> List.countBy (fst >> _.Operation) |> Map.ofList
    let expected = 4096.0/float families.Length
    Assert.Equal(families.Length, counts.Count)
    for family in families do
        Assert.InRange(float counts[family.operation], expected*0.85, expected*1.15)
    for mutation, selection in selections do
        Assert.Equal(WalkerEvolution.selectionPolicy, selection.policy)
        Assert.Equal(mutation.Operation, selection.selectedFamily)
    let s6,r6 = parent 6
    Assert.DoesNotContain(WalkerEvolution.physicalMutations s6 r6.Genome, fun m -> m.Operation = "duplicate-branch")
    let _, sixSelection = WalkerEvolution.chooseWithSelection s6 r6.Genome 42 0
    Assert.DoesNotContain(sixSelection.availableFamilies, fun family -> family.operation = "duplicate-branch")

[<Fact>]
let ``mutation sampling replays seed and attempt without fixed family slots`` () =
    let s,r = parent 2
    let families =
        WalkerEvolution.mutations s r.Genome @ WalkerEvolution.lengthMutations s r.Genome
        @ WalkerEvolution.topologyMutations s r.Genome
        @ WalkerEvolution.physicalMutations s r.Genome
        |> List.groupBy _.Operation |> List.sortBy fst
    let random = Random 42
    let sample = WalkerEvolution.chooseWithSelection s r.Genome
    for attempt in 0..63 do
        let _, choices = families[random.Next families.Length]
        let expected = choices[random.Next choices.Length]
        let mutation, selection = sample 42 attempt
        Assert.Equal(expected, mutation)
        Assert.Equal(selection, snd (sample 42 attempt))
    Assert.Equal(fst (sample 42 0), WalkerEvolution.choose s r.Genome 42 0)
    Assert.Throws<ArgumentException>(fun () -> WalkerEvolution.choose s r.Genome -1 0 |> ignore) |> ignore
    Assert.Throws<ArgumentException>(fun () -> WalkerEvolution.choose s r.Genome 42 64 |> ignore) |> ignore

[<Theory>]
[<InlineData(2)>]
[<InlineData(3)>]
[<InlineData(4)>]
[<InlineData(5)>]
[<InlineData(6)>]
let ``foot replacements preserve the terminal side and can reach below the chassis`` feet =
    let s,r = parent feet
    let edits = WalkerStructural.footEdits s r.Genome
    Assert.NotEmpty edits
    for edit in edits do
        let change = edit.Changes |> List.find (fun (path,_,_) -> path.EndsWith "/contact")
        let _,before,after = change
        Assert.Equal(100.0, before[0], 6)
        Assert.True(after[0] > 80.0, $"Contact must extend beyond the outer hinge, not the opposite end: {after}")
        if edit.Operation = "lengthen-foot" then
            Assert.True(after[0] > before[0])
            Assert.Equal(0.0, after[1], 6)
        let output = List.zip s.Program.Feet edit.Program.Feet |> List.find (fun (a,b) ->
            let source = s.Program.Bodies |> List.find (fun body -> body.Id = a.Port.Body)
            let target = edit.Program.Bodies |> List.find (fun body -> body.Id = b.Port.Body)
            source.Sites <> target.Sites) |> snd
        let site = edit.Program.Bodies |> List.find (fun b -> b.Id = output.Port.Body) |> _.Sites |> List.find (fun site -> site.Id = output.Port.Site)
        let lowest = s.Frames |> List.map (fun frame -> (Motion.transform frame.Poses[output.Port.Body] site.Position).Y+output.Radius) |> List.max
        Assert.True(lowest >= 0.0, $"Contact path remains above the chassis: {lowest}")
        Assert.Equal<MotionHinge list>(s.Program.Hinges, edit.Program.Hinges)
    for foot in s.Program.Feet do
        let bends = edits |> List.filter (fun edit -> edit.Operation = "bend-foot")
                    |> List.collect _.Changes |> List.choose (fun (path,_,after) ->
                        if path = $"feet/{foot.Id}/contact" then Some (Math.Round(after[1],6)) else None) |> Set.ofList
        Assert.Contains(-32.0, bends)
        Assert.Contains(32.0, bends)

[<Fact>]
let ``foot at an outer hinge can extend without moving the hinge`` () =
    let source,robot = parent 2
    let foot = source.Program.Feet.Head
    let body = source.Program.Bodies |> List.find (fun b -> b.Id = foot.Port.Body)
    let joint = body.Sites |> List.find (fun site -> site.Id = "joint")
    let withContact position =
        let bodies =
            source.Program.Bodies |> List.map (fun b ->
                let sites =
                    b.Sites |> List.map (fun site ->
                        if site.Id = foot.Port.Site then { site with Position = position } else site)
                if b.Id <> body.Id then b else
                    { b with Sites = sites })
        { source.Program with Bodies = bodies }
    let program = withContact joint.Position
    let screened = Realization.screen program 72 |> unwrap
    let genome = { robot.Genome with MechanismId = screened.MechanismId }
    WalkerRealization.compile screened genome |> unwrap |> ignore
    let edits =
        WalkerStructural.footEdits screened genome |> List.filter (fun edit ->
            edit.Operation = "lengthen-foot"
            && (edit.Changes |> List.exists (fun (path,_,_) -> path = $"feet/{foot.Id}/contact")))
    Assert.NotEmpty edits
    for edit in edits do
        let changedBody = edit.Program.Bodies |> List.find (fun b -> b.Id = body.Id)
        Assert.Equal(joint, changedBody.Sites |> List.find (fun site -> site.Id = joint.Id))
        Assert.Equal<MotionHinge list>(program.Hinges, edit.Program.Hinges)
        let changedFoot = changedBody.Sites |> List.find (fun site -> site.Id = foot.Port.Site)
        Assert.True(changedFoot.Position.X > joint.Position.X+1.0)
        Assert.Equal(0.0, changedFoot.Position.Y, 6)
    let inside = withContact { joint.Position with X = joint.Position.X-20.0 }
    let sharedFeet = program.Feet |> List.map (fun f -> if f.Id = foot.Id then { f with Port = { f.Port with Site = joint.Id } } else f)
    let shared = { program with Feet = sharedFeet }
    for blocked in [inside;shared] do
        let screenedBlocked = Realization.screen blocked 72 |> unwrap
        let blockedGenome = { genome with MechanismId = screenedBlocked.MechanismId }
        Assert.DoesNotContain(WalkerStructural.footEdits screenedBlocked blockedGenome, fun edit ->
            edit.Changes |> List.exists (fun (path,_,_) -> path = $"feet/{foot.Id}/contact"))

[<Fact>]
let ``foot replacement alignment is independent of hinge enumeration order`` () =
    let s,r = parent 2
    let reordered = { s with Program = { s.Program with Hinges = List.rev s.Program.Hinges } }
    let contacts source =
        WalkerStructural.footEdits source r.Genome
        |> List.collect _.Changes |> List.filter (fun (path,_,_) -> path.EndsWith "/contact") |> List.sort
    Assert.Equal<(string * float list * float list) list>(contacts s, contacts reordered)

[<Fact>]
let ``terminal feet pointing above the chassis do not get replacement proposals`` () =
    let source,robot = parent 2
    let rotate = Rotation.apply (Rotation.aroundZ Math.PI)
    let program = {
        source.Program with
            Bodies = source.Program.Bodies |> List.map (fun b ->
                { b with Initial = { Origin = rotate b.Initial.Origin; Angle = b.Initial.Angle+Math.PI } })
            Motor = { source.Program.Motor with Phase = source.Program.Motor.Phase+Math.PI }
            Transmissions = source.Program.Transmissions |> List.map (fun t ->
                { t with Phase = t.Phase+(1.0-t.Ratio)*Math.PI })
    }
    let screened = Realization.screen program 72 |> unwrap
    let placement (p: WalkerPlacement) =
        { p with Position = rotate p.Position
                 Rotation = { X = rotate p.Rotation.X; Y = rotate p.Rotation.Y; Z = rotate p.Rotation.Z } }
    let genome = {
        robot.Genome with
            MechanismId = screened.MechanismId
            Bodies = robot.Genome.Bodies |> List.map (fun b -> { b with Parts = b.Parts |> List.map placement })
            Connections = robot.Genome.Connections |> List.map (fun c -> { c with Placement = placement c.Placement })
            Mounts = robot.Genome.Mounts |> List.map placement
    }
    WalkerRealization.compile screened genome |> unwrap |> ignore
    Assert.Empty(WalkerStructural.footEdits screened genome)

[<Fact>]
let ``relative branch phase changes one foot but not the root or other branches`` () =
    let s,r = parent 2
    let _,child,_ =
        WalkerEvolution.mutations s r.Genome
        |> List.filter (fun m -> m.Operation = "branch-phase")
        |> List.choose (fun m ->
            match Realization.screen m.Program 72 with
            | Error _ -> None
            | Ok child -> Some (m,child,()))
        |> List.head
    Assert.Equal(s.Program.Motor, child.Program.Motor)
    Assert.Equal<MotionTransmission list>(s.Program.Transmissions, child.Program.Transmissions)
    let differences = s.Program.Feet |> List.map (fun foot ->
        List.zip s.Frames child.Frames |> List.map (fun (a,b) ->
            let before = Motion.position s.Graph a foot.Port |> Result.defaultWith failwith
            let after = Motion.position child.Graph b foot.Port |> Result.defaultWith failwith
            Vec3.distance before after) |> List.max)
    Assert.Single(differences |> List.filter (fun distance -> distance > 1.0)) |> ignore
    Assert.Single(differences |> List.filter (fun distance -> distance < 1e-6)) |> ignore

[<Theory>]
[<InlineData("battery-layout")>]
[<InlineData("receiver-layout")>]
[<InlineData("motor-layout")>]
let ``mounted mass relocation compiles and retains its physical hardware`` operation =
    let s,r = parent 4
    let mutation,child,robot = physicalWitness operation s r.Genome
    Assert.Single(robot.Assembly.Drives) |> ignore
    Assert.Single(robot.Assembly.Parts |> List.filter (fun p -> p.Part = "58121.dat")) |> ignore
    Assert.Single(robot.Assembly.Parts |> List.filter (fun p -> p.Part = "58119.dat")) |> ignore
    Assert.Equal(r.Genome.Speed, robot.Genome.Speed)
    Assert.Equal(r.Genome.MaxTorque, robot.Genome.MaxTorque)
    Assert.NotEqual<string>(r.AssemblyId, robot.AssemblyId)
    Assert.Equal(mutation, WalkerEvolution.physicalMutations s r.Genome |> List.find ((=) mutation))
    if operation <> "motor-layout" then
        Assert.Equal(s.Program, child.Program)
        Assert.Equal<string>(s.MechanismId, child.MechanismId)
    else
        Assert.NotEqual<string>(s.Program.Motor.Body, child.Program.Motor.Body)
        Assert.Equal<string>("shaft-4", child.Program.Motor.Body)
        Assert.Contains(robot.Genome.Connections, fun c ->
            c.Group = "bearing-4" && c.Placement.Part = "3708.dat")
        for index in [3;4;5] do
            let bush = robot.Genome.Bodies |> List.collect _.Parts |> List.find (fun p -> p.Id = $"bush-{index}-100")
            Assert.Equal(80.0,bush.Position.Z,6)
        for before,after in List.zip s.Frames child.Frames do
            for foot in s.Program.Feet do
                let a = Motion.position s.Graph before foot.Port |> Result.defaultWith failwith
                let b = Motion.position child.Graph after foot.Port |> Result.defaultWith failwith
                Assert.True(Vec3.distance a b < 1e-6)

[<Fact>]
let ``receiver relocation uses existing chassis holes and preserves every other component and motion`` () =
    let s,r = parent 4
    let receiver (genome: WalkerGenome) =
        genome.Bodies |> List.collect _.Parts |> List.find (fun p -> p.Id = "payload-receiver")
    let before = receiver r.Genome
    let pins = r.Genome.Mounts |> List.filter (fun pin -> supportsPin pin before)
    let pinIds = pins |> List.map _.Id |> Set.ofList
    let unchangedParts (genome: WalkerGenome) =
        genome.Bodies |> List.map (fun body -> { body with Parts = body.Parts |> List.filter (fun p -> p.Id <> before.Id) })
    let edits = WalkerLayout.receiverEdits s r.Genome
    Assert.InRange(edits.Length, 2, 24)
    Assert.Equal<WalkerPhysicalEdit list>(edits, WalkerLayout.receiverEdits s r.Genome)
    Assert.Contains(edits, fun edit -> (receiver edit.Genome).Rotation.Z = before.Rotation.Z)
    Assert.Contains(edits, fun edit -> (receiver edit.Genome).Rotation.Z = Vec3.scale -1.0 before.Rotation.Z)
    for edit in edits do
        let next = receiver edit.Genome
        Assert.Equal(s.Program, edit.Program)
        Assert.Equal<WalkerBodyChoice list>(unchangedParts r.Genome, unchangedParts edit.Genome)
        Assert.Equal<WalkerPivotChoice list>(r.Genome.Connections, edit.Genome.Connections)
        Assert.Equal<WalkerFootChoice list>(r.Genome.Feet, edit.Genome.Feet)
        Assert.Equal(r.Genome.MotorPart, edit.Genome.MotorPart)
        Assert.Equal(r.Genome.Speed, edit.Genome.Speed)
        Assert.Equal(r.Genome.MaxTorque, edit.Genome.MaxTorque)
        Assert.Equal(before.Position.Y, next.Position.Y)
        Assert.Equal(before.Rotation.Y, next.Rotation.Y)
        let retained = r.Genome.Mounts |> List.filter (fun pin -> not (pinIds.Contains pin.Id))
        Assert.Equal<WalkerPlacement list>(retained, edit.Genome.Mounts |> List.filter (fun pin -> not (pinIds.Contains pin.Id)))
        let anchors = (edit.Genome.Bodies |> List.find (fun b -> b.Body = r.Assembly.Chassis)).Parts
                      |> List.filter (fun p -> p.Id <> next.Id)
        let moved = edit.Genome.Mounts |> List.filter (fun pin -> pinIds.Contains pin.Id)
        Assert.Equal(2, moved.Length)
        for pin in moved do
            Assert.True(supportsPin pin next)
            Assert.Contains(anchors, fun part -> supportsPin pin part)
        WalkerRealization.compile s edit.Genome |> unwrap |> ignore
    let child = edits.Head.Genome
    Assert.NotEmpty(WalkerLayout.receiverEdits s child)
    Assert.Empty(WalkerLayout.receiverEdits s { r.Genome with Mounts = r.Genome.Mounts |> List.filter (fun pin -> pin.Id <> pins.Head.Id) })
    let noncanonicalBodies = r.Genome.Bodies |> List.map (fun body ->
        let parts = body.Parts |> List.map (fun p -> if p.Id = before.Id then { p with Id = "unsupported-receiver" } else p)
        { body with Parts = parts })
    Assert.Empty(WalkerLayout.receiverEdits s { r.Genome with Bodies = noncanonicalBodies })

[<Fact>]
let ``receiver relocation skips occupied mounting shafts without deleting existing pins`` () =
    let s,r = parent 4
    let first = WalkerLayout.receiverEdits s r.Genome |> List.head
    let receiver (genome: WalkerGenome) = genome.Bodies |> List.collect _.Parts |> List.find (fun p -> p.Id = "payload-receiver")
    let target = receiver first.Genome
    let pin = first.Genome.Mounts |> List.find (fun p -> supportsPin p target)
    let blocker = { pin with Id = "existing-frame-pin" }
    let edits = WalkerLayout.receiverEdits s { r.Genome with Mounts = blocker::r.Genome.Mounts }
    Assert.NotEmpty edits
    for edit in edits do
        Assert.NotEqual(target, receiver edit.Genome)
        Assert.Contains(blocker, edit.Genome.Mounts)

[<Fact>]
let ``battery height mutation removes or adds real supports and moves its mounting pins`` () =
    let s,r = parent 4
    let battery (genome: WalkerGenome) = genome.Bodies |> List.collect _.Parts |> List.find (fun p -> p.Id = "payload-battery")
    let before = battery r.Genome
    let edits = WalkerLayout.batteryEdits s r.Genome
    Assert.NotEmpty edits
    let lower = edits |> List.find (fun e -> (battery e.Genome).Position.Y > before.Position.Y)
    let raised = edits |> List.find (fun e -> (battery e.Genome).Position.Y < before.Position.Y)
    let supports (genome: WalkerGenome) = genome.Bodies |> List.collect _.Parts |> List.filter (fun p -> p.Id.StartsWith "battery-support-") |> List.length
    Assert.Equal(supports r.Genome-2, supports lower.Genome)
    Assert.Equal(supports r.Genome+2, supports raised.Genome)
    for edit in [lower;raised] do
        let next = battery edit.Genome
        let delta = Vec3.subtract next.Position before.Position
        let pins = r.Genome.Mounts |> List.filter (fun p -> p.Id.StartsWith "battery-mount-")
        Assert.Equal(4,pins.Length)
        for pin in pins do
            let changed = edit.Genome.Mounts |> List.find (fun p -> p.Id = pin.Id)
            Assert.True(Vec3.distance changed.Position (Vec3.add pin.Position delta) < 1e-6)
        WalkerRealization.compile s { edit.Genome with MechanismId = s.MechanismId } |> unwrap |> ignore

[<Fact>]
let ``motor layout never reroots across a reduction or onto a crank shaft`` () =
    let s,r = parent 4
    for edit in WalkerLayout.motorEdits s r.Genome do
        Assert.Contains(edit.Program.Motor.Body, ["shaft-0";"shaft-4"])
    let _,reduced,robot = physicalWitness "gear-reduction" s r.Genome
    for edit in WalkerLayout.motorEdits reduced robot.Genome do
        Assert.Equal<string>(reduced.Program.Motor.Body, edit.Program.Motor.Body)

[<Fact>]
let ``motor relocation remains editable without abandoned mounting pins`` () =
    let s,r = parent 4
    let _,child,robot = physicalWitness "motor-layout" s r.Genome
    let _,_,next = physicalWitness "motor-layout" child robot.Genome
    Assert.Single(next.Assembly.Parts |> List.filter (fun p -> p.Id = "motor-bracket")) |> ignore
    Assert.Equal(2,next.Genome.Mounts |> List.filter (fun p -> p.Id.StartsWith "motor-brace-") |> List.length)

[<Fact>]
let ``motor relocation chooses a free bracket hole without removing an existing frame pin`` () =
    let s,r = parent 4
    let mutation,_,_ = physicalWitness "motor-layout" s r.Genome
    let target = mutation.Physical |> Option.get
    let brace = target.Mounts |> List.find (fun p -> p.Id = "motor-brace-left")
    let existing = { brace with Id = "existing-frame-pin" }
    let occupied = { r.Genome with Mounts = existing::r.Genome.Mounts }
    WalkerRealization.compile s occupied |> unwrap |> ignore
    let motor (genome: WalkerGenome) = genome.Bodies |> List.collect _.Parts |> List.find (fun p -> p.Id = genome.MotorPart)
    let expected = motor target
    let edits = WalkerLayout.motorEdits s occupied |> List.filter (fun edit -> motor edit.Genome = expected)
    Assert.NotEmpty edits
    for edit in edits do
        let child = Realization.screen edit.Program 72 |> unwrap
        WalkerRealization.compile child { edit.Genome with MechanismId = child.MechanismId } |> unwrap |> ignore
        Assert.Contains(existing, edit.Genome.Mounts)
        let moved = edit.Genome.Mounts |> List.find (fun p -> p.Id = brace.Id)
        Assert.Equal(20.0, Vec3.distance brace.Position moved.Position, 6)
        Assert.False(Catalog.shaftsOverlap existing.Part existing.Position existing.Rotation moved.Part moved.Position moved.Rotation)

[<Theory>]
[<InlineData(2)>]
[<InlineData(3)>]
[<InlineData(4)>]
[<InlineData(5)>]
[<InlineData(6)>]
let ``native mutation replay changes motion and preserves physical robot policy`` feet =
    let s, r = parent feet
    let mutation = parameterWitness s r.Genome
    Assert.Equal(mutation, parameterWitness s r.Genome)
    Assert.NotEqual(s.Program, mutation.Program)
    let child = Realization.screen mutation.Program 72 |> unwrap
    let variants = WalkerEvolution.realize s r.Genome mutation child 2 |> unwrap
    Assert.Equal(2, variants.Length)
    let robots = variants |> List.map (WalkerRealization.compile child >> unwrap)
    Assert.NotEqual<string>(r.GenotypeId, robots.Head.GenotypeId)
    Assert.NotEqual<string>(robots.Head.AssemblyId, robots[1].AssemblyId)
    Assert.Equal(robots.Head.GenotypeId, (WalkerRealization.compile child variants.Head |> unwrap).GenotypeId)
    for robot in robots do
        Assert.Equal(2*feet, robot.Assembly.Feet.Length)
        Assert.Single(robot.Assembly.Drives) |> ignore
        Assert.True(robot.Genome.Remote)
        Assert.False(robot.Genome.TowWagon)
        Assert.Equal(s.Program.Bodies.Length, child.Program.Bodies.Length)
        Assert.Equal(s.Program.Hinges.Length, child.Program.Hinges.Length)
    Assert.True(child.Frames |> List.forall (fun f -> f.MaxResidual < 1e-7))

[<Theory>]
[<InlineData("move-hinge-site")>]
[<InlineData("input-phase")>]
[<InlineData("branch-phase")>]
let ``each supported operator can produce a catalog realized motion edit`` operation =
    let s, r = parent 4
    let supported =
        WalkerEvolution.mutations s r.Genome
        |> List.filter (fun m -> m.Operation = operation)
        |> List.choose (fun m ->
            match Realization.screen m.Program 72 with
            | Error _ -> None
            | Ok child ->
                match WalkerEvolution.realize s r.Genome m child 1 with
                | Error _ -> None
                | Ok [genome] ->
                    match WalkerRealization.compile child genome with
                    | Ok robot -> Some (m, child, robot)
                    | Error _ -> None
                | Ok _ -> failwith "Expected one physical realization.")
    Assert.NotEmpty supported
    let mutation, child, robot = supported.Head
    Assert.NotEqual<string>(s.MechanismId, child.MechanismId)
    Assert.True(mutation.Changes |> List.forall (fun c -> c.Before <> c.After))
    if operation = "branch-phase" then
        let changed, _ = mutation.RotatedPart.Value
        let source = r.Genome.Bodies |> List.collect _.Parts
        let target = robot.Genome.Bodies |> List.collect _.Parts
        let changedPart = target |> List.find (fun p -> p.Id = changed)
        let oldPart = source |> List.find (fun p -> p.Id = changed)
        Assert.True(Vec3.distance changedPart.Rotation.X oldPart.Rotation.X > 1.0)
        for part in source |> List.filter (fun p -> p.Part = "3648b.dat") do
            let next = target |> List.find (fun p -> p.Id = part.Id)
            Assert.True(Vec3.distance part.Rotation.X next.Rotation.X < 1e-6)
    if operation = "input-phase" then
        Assert.NotEqual(s.Program.Motor.Phase, child.Program.Motor.Phase)
    if operation = "move-hinge-site" then
        let change = mutation.Changes.Head
        Assert.Equal(20.0, abs (change.After.Head-change.Before.Head), 6)

[<Fact>]
let ``physical pin alternatives reverse a compatible short pin and preserve its axial extent`` () =
    let s, r = parent 4
    let mutation = parameterWitness s r.Genome
    let child = Realization.screen mutation.Program 72 |> unwrap
    let choices = WalkerEvolution.realize s r.Genome mutation child 2 |> unwrap
    let before = choices.Head.Connections
    let after = choices[1].Connections
    let source, target = List.zip before after |> List.find (fun (a,b) -> a <> b)
    Assert.Equal("32002.dat", source.Placement.Part)
    Assert.Equal("32002.dat", target.Placement.Part)
    Assert.True(Vec3.distance
        (Vec3.subtract source.Placement.Position (Vec3.scale 10.0 source.Placement.Rotation.X))
        target.Placement.Position < 1e-6)
    Assert.Equal(Vec3.scale -1.0 source.Placement.Rotation.X, target.Placement.Rotation.X)
    Assert.Throws<ArgumentException>(fun () -> WalkerEvolution.choose s r.Genome 42 -1 |> ignore) |> ignore
    Assert.Throws<ArgumentException>(fun () -> WalkerEvolution.realize s r.Genome mutation child 5 |> ignore) |> ignore
    let unsupported = { r.Genome with Mounts = { r.Genome.Mounts.Head with Position = Vec3.create 999 999 999 } :: r.Genome.Mounts.Tail }
    Assert.True((WalkerEvolution.realize s unsupported mutation child 1).IsError)

[<Fact>]
let ``construction alternatives preserve the mechanism and change only one passive pivot`` () =
    let s, r = parent 2
    let choices = WalkerConstruction.alternatives s r.Genome 4
    Assert.Equal(4, choices.Length)
    Assert.Equal(r.Genome, choices.Head.Genome)
    for choice in choices.Tail do
        Assert.Equal(s.MechanismId, choice.Genome.MechanismId)
        Assert.Equal<WalkerBodyChoice list>(r.Genome.Bodies, choice.Genome.Bodies)
        Assert.Equal<WalkerPlacement list>(r.Genome.Mounts, choice.Genome.Mounts)
        Assert.Equal<WalkerFootChoice list>(r.Genome.Feet, choice.Genome.Feet)
        Assert.Equal(r.Genome.Speed, choice.Genome.Speed)
        Assert.Equal(r.Genome.MaxTorque, choice.Genome.MaxTorque)
        Assert.Equal(1, List.zip r.Genome.Connections choice.Genome.Connections |> List.filter (fun (a,b) -> a <> b) |> List.length)
        Assert.True((WalkerRealization.compile s choice.Genome).IsOk)
        Assert.NotEmpty choice.Members
    Assert.Single(WalkerConstruction.alternatives s r.Genome 1) |> ignore
    Assert.Throws<ArgumentException>(fun () -> WalkerConstruction.alternatives s r.Genome 33 |> ignore) |> ignore

[<Fact>]
let ``construction prefers a shorter compatible pin instead of arbitrary extension`` () =
    let s, r = parent 2
    let original = r.Genome.Connections |> List.find (fun p -> p.Placement.Part = "32002.dat")
    let placement = {
        original.Placement with
            Part = "3673.dat"
            Position = Vec3.subtract original.Placement.Position (Vec3.scale 5.0 original.Placement.Rotation.X)
    }
    let longer = { original with Placement = placement }
    let genome = { r.Genome with Connections = r.Genome.Connections |> List.map (fun c -> if c.Group = original.Group then longer else c) }
    let repair = WalkerConstruction.choices s genome |> Seq.find (fun c -> c.Pivot = original.Group)
    Assert.Equal("smooth-pivot-shorter-pin", repair.Motif)
    let pin = repair.Genome.Connections |> List.find (fun c -> c.Group = original.Group) |> _.Placement
    Assert.Equal(original.Placement, pin)

[<Fact>]
let ``construction never shortens a three body shared pivot below connector contact capacity`` () =
    let p,g = Walker1Seeds.walker 2
    let s = Realization.screen p 72 |> unwrap
    let r = WalkerRealization.compile s g |> unwrap
    let pivot = r.Genome.Connections |> List.find (fun c -> c.Group = "leg-0-shared-upper-pivot")
    Assert.Equal("32556.dat", pivot.Placement.Part)
    let choices = WalkerConstruction.choices s r.Genome |> Seq.filter (fun c -> c.Pivot = pivot.Group) |> Seq.toList
    Assert.NotEmpty choices
    for choice in choices do
        Assert.Equal(3, choice.Members.Length)
        let pin = choice.Genome.Connections |> List.find (fun c -> c.Group = pivot.Group)
        Assert.Equal("32556.dat", pin.Placement.Part)
    Assert.Equal(r.Genome, (WalkerConstruction.alternatives s r.Genome 1).Head.Genome)

[<Fact>]
let ``construction prioritizes an obstructed pin over unrelated compatible alternatives`` () =
    let s, r = parent 2
    let original = r.Genome.Connections |> List.find (fun p -> p.Group = "leg-0-0-crank")
    let placement = {
        original.Placement with
            Part = "3673.dat"
            Position = Vec3.subtract original.Placement.Position (Vec3.scale 5.0 original.Placement.Rotation.X)
    }
    let longer = { original with Placement = placement }
    let genome = { r.Genome with Connections = r.Genome.Connections |> List.map (fun c -> if c.Group = original.Group then longer else c) }
    let choices = WalkerConstruction.alternatives s genome 2
    let repair = choices[1]
    Assert.Equal(genome, choices.Head.Genome)
    Assert.Equal(original.Group, repair.Pivot)
    Assert.True(repair.ClearanceImprovement > 0.0)
    Assert.Equal(original, repair.Genome.Connections |> List.find (fun c -> c.Group = original.Group))
    Assert.Equal<WalkerConstructionChoice list>(choices, WalkerConstruction.alternatives s genome 2)
    let report = WalkerConstruction.describe s genome repair.Genome
    Assert.Equal(repair.ClearanceImprovement, report.ClearanceImprovement)

[<Fact>]
let ``diagnostic layer repair preserves motion and keeps full thin-link socket engagement`` () =
    let s,r = parent 2
    let body = r.Genome.Bodies |> List.find (fun b -> b.Body = "leg-0-1-coupler")
    let feedback = { Kind = "rigid-body-partition"; Parts = [body.Parts.Head.Id]; Bodies = ["shaft-2"] }
    let repair = WalkerConstruction.repairs s r.Genome feedback 1 |> List.exactlyOne
    Assert.Equal("passive-link-layer-clearance", repair.Motif)
    Assert.Equal<string list>([body.Body], repair.AffectedBodies)
    Assert.Equal(10.0, repair.LayerOffset)
    Assert.Equal(s.MechanismId, repair.Genome.MechanismId)
    Assert.Equal<WalkerPivotChoice list>(r.Genome.Connections, repair.Genome.Connections)
    Assert.Equal<WalkerPlacement list>(r.Genome.Mounts, repair.Genome.Mounts)
    for before,after in List.zip r.Genome.Bodies repair.Genome.Bodies do
        if before.Body <> body.Body then Assert.Equal(before,after)
        else
            for a,b in List.zip before.Parts after.Parts do
                Assert.Equal(a.Position.X,b.Position.X)
                Assert.Equal(a.Position.Y,b.Position.Y)
                Assert.Equal(a.Position.Z+10.0,b.Position.Z)
                Assert.Equal(a.Rotation,b.Rotation)
    for before,after in List.zip r.Genome.Feet repair.Genome.Feet do
        Assert.Equal(before.Depth+(if before.Part = body.Parts.Head.Id then 10.0 else 0.0),after.Depth)
    Assert.True((WalkerRealization.compile s repair.Genome).IsOk)
    Assert.DoesNotContain(WalkerConstruction.repairs s r.Genome feedback 32,
                          fun choice -> abs choice.LayerOffset > 10.0)

[<Fact>]
let ``diagnostic repair excludes driven bodies and unrelated passive pivots`` () =
    let s,r = parent 2
    Assert.Empty(WalkerConstruction.repairs s r.Genome
        { Kind = "rigid-body-partition"; Parts = []; Bodies = ["shaft-2";"chassis"] } 2)
    let pin = r.Genome.Connections |> List.find (fun c -> c.Group = "leg-0-1-crank")
    let feedback = { Kind = "initial-self-interference"; Parts = [pin.Placement.Id]; Bodies = [] }
    let choices = WalkerConstruction.repairs s r.Genome feedback 2
    Assert.NotEmpty choices
    Assert.All(choices, fun choice -> Assert.Equal(pin.Group,choice.Pivot))
    Assert.Throws<ArgumentException>(fun () ->
        WalkerConstruction.repairs s r.Genome { Kind = "unknown"; Parts = []; Bodies = [] } 1 |> ignore) |> ignore

[<Fact>]
let ``builder catalogue and policy identity are explicit and stable`` () =
    let policy = WalkerBuilder.defaultPolicy
    Assert.Equal<string list>(WalkerBuilder.catalogue |> List.map _.Id, policy.MotifOrder)
    Assert.All(WalkerBuilder.catalogue, fun entry ->
        Assert.Equal("hand-authored", entry.Origin)
        Assert.NotEmpty entry.Preconditions
        Assert.NotEmpty entry.Preserves
        Assert.NotEmpty entry.Limitations
        Assert.NotEmpty entry.Evidence)
    Assert.Equal("8d83e7061a7c6833d43d166759e59e41b8f233ef8a02030a62ff14d0f15b6acb", WalkerBuilder.policyId policy)
    let path = Path.GetTempFileName()
    try
        File.WriteAllText(path, Realization.json policy)
        Assert.Equal(policy, WalkerBuilder.readPolicy path)
        let malformed = System.Text.Json.Nodes.JsonNode.Parse(Realization.json policy).AsObject()
        malformed.Remove("clearanceFirst") |> ignore
        let missing = malformed.ToJsonString()
        malformed["clearanceFirst"] <- System.Text.Json.Nodes.JsonValue.Create(true)
        malformed["extra"] <- System.Text.Json.Nodes.JsonValue.Create(1)
        for text in ["null"; "{}"; missing; malformed.ToJsonString()] do
            File.WriteAllText(path, text)
            Assert.ThrowsAny<Exception>(fun () -> WalkerBuilder.readPolicy path |> ignore) |> ignore
    finally File.Delete path
    for invalid in [
        { policy with SchemaVersion = 2 }
        { policy with MotifOrder = [policy.MotifOrder.Head;policy.MotifOrder.Head] }
        { policy with PinOrder = ["unsafe.dat";"3673.dat";"32556.dat"] }
        { policy with LayerOffsets = [10;-10;20;999] }
    ] do
        Assert.Throws<ArgumentException>(fun () -> WalkerBuilder.validate invalid |> ignore) |> ignore

[<Fact>]
let ``default builder policy preserves construction identities and ordering`` () =
    let s,r = parent 2
    let direct = WalkerConstruction.alternatives s r.Genome 4
    let configured = WalkerConstruction.alternativesWithPolicy WalkerBuilder.defaultPolicy s r.Genome 4
    Assert.Equal<WalkerConstructionChoice list>(direct, configured)
    let mutation = parameterWitness s r.Genome
    let child = Realization.screen mutation.Program 72 |> unwrap
    Assert.Equal<WalkerGenome list>(
        WalkerEvolution.realize s r.Genome mutation child 4 |> unwrap,
        WalkerEvolution.realizeWithPolicy WalkerBuilder.defaultPolicy s r.Genome mutation child 4 |> unwrap)

[<Fact>]
let ``builder policy reorders supported repairs without weakening construction checks`` () =
    let s,r = parent 2
    let body = r.Genome.Bodies |> List.find (fun b -> b.Body = "leg-0-1-coupler")
    let feedback = { Kind = "rigid-body-partition"; Parts = [body.Parts.Head.Id]; Bodies = ["shaft-2"] }
    let outward = WalkerConstruction.repairs s r.Genome feedback 1 |> List.exactlyOne
    let policy = { WalkerBuilder.defaultPolicy with LayerOffsets = [-10;10;-20;20] }
    let inward = WalkerConstruction.repairsWithPolicy policy s r.Genome feedback 1 |> List.exactlyOne
    Assert.Equal(10.0, outward.LayerOffset)
    Assert.Equal(-10.0, inward.LayerOffset)
    Assert.NotEqual<string>(WalkerBuilder.policyId WalkerBuilder.defaultPolicy, WalkerBuilder.policyId policy)
    Assert.Equal(inward, WalkerConstruction.repairsWithPolicy policy s r.Genome feedback 1 |> List.exactlyOne)
    Assert.True((WalkerRealization.compile s inward.Genome).IsOk)
    Assert.Throws<ArgumentException>(fun () ->
        WalkerConstruction.repairsWithPolicy policy s r.Genome feedback 33 |> ignore) |> ignore
    let pin = r.Genome.Connections |> List.find (fun c -> c.Group = "leg-0-1-crank")
    let mixed = { feedback with Parts = pin.Placement.Id :: feedback.Parts }
    let pinFirst = WalkerConstruction.repairs s r.Genome mixed 1 |> List.exactlyOne
    let layerFirst = { policy with MotifOrder = List.rev policy.MotifOrder }
    let ordered = WalkerConstruction.repairsWithPolicy layerFirst s r.Genome mixed 1 |> List.exactlyOne
    Assert.NotEqual<string>("passive-link-layer-clearance", pinFirst.Motif)
    Assert.Equal("passive-link-layer-clearance", ordered.Motif)

[<Fact>]
let ``motif gallery exports compiler checked witnesses for every catalogue entry`` () =
    let directory = Path.Combine(Path.GetTempPath(), "lego-motif-gallery-"+Guid.NewGuid().ToString("N"))
    try
        WalkerMotifGallery.write directory
        use gallery = System.Text.Json.JsonDocument.Parse(File.ReadAllText(Path.Combine(directory, "gallery.json")))
        let examples = gallery.RootElement.GetProperty("examples").EnumerateArray() |> Seq.toList
        Assert.Equal(3, examples.Length)
        Assert.Equal<Set<string>>(WalkerBuilder.catalogue |> List.map _.Id |> Set.ofList,
                                 examples |> List.map (fun e -> e.GetProperty("motif").GetString()) |> Set.ofList)
        for example in examples do
            let id = example.GetProperty("id").GetString()
            Assert.NotEqual<string>(example.GetProperty("beforeGenotypeId").GetString(), example.GetProperty("afterGenotypeId").GetString())
            let parts (key: string) = example.GetProperty(key).EnumerateArray() |> Seq.toList
            Assert.Equal<string list>(parts "before" |> List.map (fun p -> p.GetProperty("id").GetString()),
                                      parts "after" |> List.map (fun p -> p.GetProperty("id").GetString()))
            Assert.Single(parts "after" |> List.filter (fun p -> p.GetProperty("changed").GetBoolean())) |> ignore
            let before = WalkerRealization.readGenome (Path.Combine(directory,id,"before","realization.json"))
            let after = WalkerRealization.readGenome (Path.Combine(directory,id,"after","realization.json"))
            Assert.Equal(before.MechanismId, after.MechanismId)
            Assert.Equal(before.Speed, after.Speed)
            Assert.Equal(before.MaxTorque, after.MaxTorque)
        Assert.Throws<ArgumentException>(fun () -> WalkerMotifGallery.write directory) |> ignore
    finally
        if Directory.Exists directory then Directory.Delete(directory,true)

[<Fact>]
let ``construction never introduces a contact with an undeclared pivot body`` () =
    let s, r = parent 2
    let pivot = r.Genome.Connections |> List.find (fun c -> c.Placement.Part = "32002.dat")
    let expected =
        s.Program.Hinges |> List.filter (fun h -> List.contains h.Id pivot.Hinges)
        |> List.collect (fun h -> [h.A.Body;h.B.Body]) |> Set.ofList
    let other = r.Genome.Bodies |> List.find (fun b -> not (expected.Contains b.Body))
    let lo,hi = Catalog.shaftRange pivot.Placement.Part
    let extra = {
        Id = "unintended-bearing"; Part = "32523.dat"; Color = 0
        Position = Vec3.add pivot.Placement.Position (Vec3.scale ((lo+hi)/2.0) pivot.Placement.Rotation.X)
        Rotation = Rotation.beamWithNormal pivot.Placement.Rotation.X (Vec3.create 1 0 0)
    }
    let genome = { r.Genome with Bodies = r.Genome.Bodies |> List.map (fun b -> if b.Body = other.Body then { b with Parts = extra::b.Parts } else b) }
    Assert.DoesNotContain(WalkerConstruction.choices s genome, fun c -> c.Pivot = pivot.Group)

[<Fact>]
let ``native motion mutation is an inspectable gallery snapshot`` () =
    let s, r = parent 4
    let mutation = parameterWitness s r.Genome
    let child = Realization.screen mutation.Program 72 |> unwrap
    let svg = MotionExport.cycleSvg child.Program child.Graph child.Frames
    let snapshot = Path.Combine(AppContext.BaseDirectory, "visualizations", "native-mutation.svg")
    Assert.Equal(File.ReadAllText(snapshot).Replace("\r\n", "\n"), svg.Replace("\r\n", "\n"))

[<Fact>]
let ``catalog dyad insertion and removal change connectivity and reattach the foot`` () =
    let s, r = parent 4
    let random = Random 42
    let mutation =
        WalkerEvolution.topologyMutations s r.Genome
        |> List.map (fun mutation -> random.Next(), mutation) |> List.sortBy fst
        |> List.item 3 |> snd
    Assert.Equal("add-dyad", mutation.Operation)
    let child = Realization.screen mutation.Program 72 |> unwrap
    let snapshot = Path.Combine(AppContext.BaseDirectory, "visualizations", "native-dyad.svg")
    Assert.Equal(File.ReadAllText(snapshot).Replace("\r\n", "\n"),
                 (MotionExport.cycleSvg child.Program child.Graph child.Frames).Replace("\r\n", "\n"))
    Assert.Equal(s.Program.Bodies.Length+2, child.Program.Bodies.Length)
    Assert.Equal(s.Program.Hinges.Length+3, child.Program.Hinges.Length)
    Assert.Equal(Motion.footCount s.Program, Motion.footCount child.Program)
    let genome = WalkerEvolution.realize s r.Genome mutation child 1 |> unwrap |> List.exactlyOne
    let robot = WalkerRealization.compile child genome |> unwrap
    Assert.True(robot.Groups |> List.exists (fun (_,hinges,_) -> hinges.Length = 2))
    let removal = WalkerEvolution.topologyMutations child robot.Genome |> List.find (fun m -> m.Operation = "remove-dyad")
    let smaller = Realization.screen removal.Program 72 |> unwrap
    let concrete = WalkerEvolution.realize child robot.Genome removal smaller 1 |> unwrap |> List.exactlyOne
    let result = WalkerRealization.compile smaller concrete |> unwrap
    Assert.Equal(s.Program.Bodies.Length, smaller.Program.Bodies.Length)
    Assert.Equal(s.Program.Hinges.Length, smaller.Program.Hinges.Length)
    Assert.Equal(8, result.Assembly.Feet.Length)
    Assert.Single(result.Assembly.Drives) |> ignore

[<Fact>]
let ``Walker1 linkage can be constructed as two primitive dyad additions to a four bar`` () =
    let names = Set.ofList ["ground"; "crank"; "upper"; "long-leg"]
    let core = {
        MotionExamples.walker1 with
            Bodies = MotionExamples.walker1.Bodies |> List.filter (fun b -> names.Contains b.Id)
            Hinges = MotionExamples.walker1.Hinges |> List.filter (fun h -> names.Contains h.A.Body && names.Contains h.B.Body)
            Feet = [Motion.foot "sole" "long-leg" "tip" 4]
    }
    let first = Motion.addDyad "cross" (Motion.port "crank" "tip") (Motion.port "upper" "root") 40 80 1 core
                |> Result.defaultWith failwith
    let extendedBodies = first.Bodies |> List.map (fun b ->
        if b.Id = "cross-a" then { b with Sites = b.Sites @ [Motion.site "left" -40 0] } else b)
    let extended = { first with Bodies = extendedBodies }
    let reference = Realization.screen MotionExamples.walker1 72 |> unwrap
    let matches = [
        for branch in [-1; 1] do
            let next = Motion.addDyad "sole" (Motion.port "long-leg" "tip") (Motion.port "cross-a" "left") 40 120 branch extended
                       |> Result.defaultWith failwith
            let program = {
                next with
                    Bodies = next.Bodies |> List.map (fun b ->
                        if b.Id = "sole-a" then { b with Sites = b.Sites @ [Motion.site "contact" 20 -14] } else b)
                    Feet = [Motion.foot "sole" "sole-a" "contact" 4]
            }
            match Realization.screen program 72 with
            | Error _ -> ()
            | Ok solved ->
                let errors = List.zip reference.Frames solved.Frames |> List.map (fun (a,b) ->
                    let pa = (Motion.footPositions reference.Graph a).["sole"]
                    let pb = (Motion.footPositions solved.Graph b).["sole"]
                    Vec3.distance pa pb)
                yield List.max errors
    ]
    Assert.Contains(matches, fun error -> error < 1e-5)

[<Fact>]
let ``whole Walker1 seed is an inspectable gallery snapshot`` () =
    let program, _ = Walker1Seeds.walker 2
    let s = Realization.screen program 72 |> unwrap
    let snapshot = Path.Combine(AppContext.BaseDirectory, "visualizations", "walker1-whole-walker.svg")
    Assert.Equal(File.ReadAllText(snapshot).Replace("\r\n", "\n"),
                 (MotionExport.cycleSvg s.Program s.Graph s.Frames).Replace("\r\n", "\n"))
