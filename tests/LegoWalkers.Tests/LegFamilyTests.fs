module LegoWalkers.LegFamilyTests

open System
open System.IO
open LegoWalkers
open Xunit

let private unwrap result = Result.defaultWith (Realization.json >> failwith) result

let private founder feet =
    let program,genome = WalkerSeeds.sharedFourBar feet
    let screened = Realization.screen program 72 |> unwrap
    screened,WalkerRealization.compile screened genome |> unwrap

[<Fact>]
let ``shared founders retain mirror and half-cycle arrangement`` () =
    let screened,walker = founder 4
    let family = walker.Genome.LegFamilies.Value |> List.exactlyOne
    Assert.Equal(4,family.Instances.Length)
    Assert.Equal<bool list>([false;true;false;true],family.Instances |> List.map _.Mirrored)
    Assert.Equal<float list>([0.0;Math.PI;Math.PI;0.0],family.Instances |> List.map _.Phase)
    Assert.Single(family.Instances |> List.map (WalkerLegFamilies.motionSignature screened.Program) |> List.distinct) |> ignore
    let p,g = Walker1Seeds.sharedWalker 2
    WalkerRealization.compile (Realization.screen p 72 |> unwrap) g |> unwrap |> ignore

[<Fact>]
let ``old recipes roundtrip without family metadata or changing identity`` () =
    let program,genome = WalkerSeeds.fourBar 2
    let screened = Realization.screen program 72 |> unwrap
    let before = WalkerRealization.compile screened genome |> unwrap
    let file = Path.GetTempFileName()
    try
        let json = WalkerRealization.json genome
        Assert.DoesNotContain("legFamilies",json)
        File.WriteAllText(file,json)
        let decoded = WalkerRealization.readGenome file
        Assert.True(decoded.LegFamilies.IsNone)
        let after = WalkerRealization.compile screened decoded |> unwrap
        Assert.Equal(before.GenotypeId,after.GenotypeId)
    finally File.Delete file

[<Fact>]
let ``legacy conversion binds exact copies and preserves asymmetry and explicit splits`` () =
    let program,genome = WalkerSeeds.fourBar 4
    let screened = Realization.screen program 72 |> unwrap
    let converted = WalkerLegFamilies.migrate screened genome
    Assert.Equal("converted",converted.Status)
    Assert.Equal(4,converted.Families.Head.Instances.Length)
    Assert.Empty(converted.Unmatched)
    Assert.Equal(genome,{converted.Genome with LegFamilies=None})
    let feet =
        program.Feet |> List.mapi (fun n foot ->
            if n=0 then {foot with Radius=foot.Radius+1.0} else foot)
    let asymmetric = {screened with Program={program with Feet=feet}}
    let partial = WalkerLegFamilies.migrate asymmetric genome
    Assert.Equal(3,partial.Families.Head.Instances.Length)
    Assert.Equal(program.Feet.Head.Id,(Assert.Single partial.Unmatched).Foot)
    let split = WalkerLegFamilies.split converted.Families.Head.Id converted.Families.Head.Instances.Head.Id converted.Genome
    let repeated = WalkerLegFamilies.migrate screened split
    Assert.Equal("existing-constraints-preserved",repeated.Status)
    Assert.Equal(split,repeated.Genome)

[<Fact>]
let ``compiler rejects unilateral edits to a shared template and its gait`` () =
    let screened,walker = founder 4
    let independent = {walker.Genome with LegFamilies=None}
    for operation in ["move-hinge-site";"branch-phase"] do
        let mutation = WalkerEvolution.mutations screened independent |> List.find (fun m -> m.Operation=operation)
        let child = Realization.screen mutation.Program 72 |> unwrap
        let physical = WalkerEvolution.realize screened independent mutation child 1 |> unwrap |> List.head
        Assert.True((WalkerRealization.compile child {physical with LegFamilies=walker.Genome.LegFamilies}).IsError)
    let instance = walker.Genome.LegFamilies.Value.Head.Instances.Head
    let drive = instance.Attachments |> List.find (fun a -> a.Role="drive") |> _.Port
    let changed =
        {screened.Program with
            Bodies=screened.Program.Bodies |> List.map (fun b ->
                if b.Id<>drive.Body then b else
                    {b with
                        Sites=b.Sites |> List.map (fun site ->
                            if site.Id=drive.Site then {site with Position=Vec3.scale 2.0 site.Position} else site)})}
    Assert.Throws<ArgumentException>(fun () ->
        WalkerLegFamilies.validateAt changed screened.Frames.Head.Poses walker.Genome) |> ignore

[<Fact>]
let ``shared leg mutations change all four copies without changing mounting or relative phases`` () =
    let screened,walker = founder 4
    let edits = WalkerEvolution.editsWithPolicy WalkerBuilder.defaultPolicy screened walker.Genome
    Assert.DoesNotContain(edits,fun (_,_,m) -> m.Operation="branch-phase")
    let outcomes =
        edits |> List.choose (fun (_,_,m) ->
            if m.Operation<>"shared-leg-lengthen-foot" then None else
            let result =
                Realization.screen m.Program 72 |> Result.bind (fun child ->
                    WalkerEvolution.realize screened walker.Genome m child 1
                    |> Result.bind (fun choices -> WalkerRealization.compile child choices.Head)
                    |> Result.map (fun robot -> child,robot))
            Some result)
    let successes = outcomes |> List.choose (function Ok value -> Some value | _ -> None)
    let errors = outcomes |> List.choose (function Error error -> Some (Realization.json error) | _ -> None)
    Assert.True(not successes.IsEmpty,String.concat "\n" errors)
    let child,robot = successes.Head
    let before = walker.Genome.LegFamilies.Value.Head.Instances
    let after = robot.Genome.LegFamilies.Value.Head.Instances
    Assert.Equal<LegInstance list>(before,after)
    Assert.Equal<MotionTransmission list>(screened.Program.Transmissions,child.Program.Transmissions)
    for instance in after do
        let oldFoot = screened.Program.Feet |> List.find (fun f -> f.Id=instance.Foot)
        let newFoot = child.Program.Feet |> List.find (fun f -> f.Id=instance.Foot)
        let site (program: MotionProgram) (port: MotionPort) =
            (program.Bodies |> List.find (fun b -> b.Id=port.Body)).Sites |> List.find (fun s -> s.Id=port.Site)
        Assert.NotEqual((site screened.Program oldFoot.Port).Position,(site child.Program newFoot.Port).Position)

[<Fact>]
let ``splitting changes only the genotype constraint and leaves three shared legs`` () =
    let screened,walker = founder 4
    let old = walker.Genome.LegFamilies.Value.Head
    let split = WalkerLegFamilies.split old.Id old.Instances.Head.Id walker.Genome
    let child = WalkerRealization.compile screened split |> unwrap
    Assert.Equal(walker.AssemblyId,child.AssemblyId)
    Assert.NotEqual<string>(walker.GenotypeId,child.GenotypeId)
    Assert.Equal<int list>([1;3],split.LegFamilies.Value |> List.map (fun f -> f.Instances.Length) |> List.sort)
    let splitAgain = WalkerLegFamilies.split old.Id old.Instances.Tail.Head.Id split
    Assert.Equal(3,splitAgain.LegFamilies.Value.Length)

[<Fact>]
let ``outward L linkage embeds as a shared family and grows four actual legs`` () =
    let program,genome = Walker1Seeds.walker 2
    let original = Realization.screen program 72 |> unwrap
    let first = WalkerLevelFeet.edits original genome |> List.head
    let one = Realization.screen first.Program 72 |> unwrap
    let changed = one.Program.Feet |> List.find (fun f -> f.Port.Body.EndsWith("-sole-output")) |> _.Id
    let second =
        WalkerLevelFeet.edits one first.Genome
        |> List.find (fun e -> e.Changes |> List.exists (fun (path,_,_) ->
            path.EndsWith("/level-loop") && path<> $"feet/{changed}/level-loop"))
    let two = Realization.screen second.Program 72 |> unwrap
    let shared = WalkerLegFamilies.adopt "L-legs" (two.Program.Feet |> List.map _.Id) two.Program second.Genome
    let robot = WalkerRealization.compile two shared |> unwrap
    let grow (parent: ScreenedMotion) (walker: RealizedWalker) =
        let attempts =
            WalkerEvolution.editsWithPolicy WalkerBuilder.defaultPolicy parent walker.Genome
            |> List.filter (fun (_,_,m) -> m.Operation="duplicate-branch")
            |> List.map (fun (_,_,m) ->
                Realization.screen m.Program 72 |> Result.bind (fun child ->
                    WalkerEvolution.realize parent walker.Genome m child 1
                    |> Result.bind (fun choices -> WalkerRealization.compile child choices.Head)
                    |> Result.map (fun r -> child,r)))
        let successes = attempts |> List.choose (function Ok x -> Some x | _ -> None)
        let errors = attempts |> List.choose (function Error e -> Some (Realization.json e) | _ -> None)
        Assert.True(not successes.IsEmpty,String.concat "\n" errors)
        successes.Head
    let three,r3 = grow two robot
    let four,r4 = grow three r3
    Assert.Equal(4,four.Program.Feet.Length)
    Assert.Equal(4,r4.Genome.LegFamilies.Value.Head.Instances.Length)
    Assert.Equal(2,r4.Genome.LegFamilies.Value.Head.Instances |> List.filter _.Mirrored |> List.length)
    Assert.Equal(4,r4.Assembly.Parts |> List.filter (fun p -> p.Part="32526.dat") |> List.length)
    Assert.All(four.Program.Feet,fun foot ->
        Assert.All(four.Frames,fun frame ->
            Assert.InRange(abs frame.Poses[foot.Port.Body].Angle,0.0,1e-6)))
    let catalogue = WalkerEvolution.editsWithPolicy WalkerBuilder.defaultPolicy four r4.Genome
    let _,_,hinge =
        catalogue |> List.find (fun (_,_,m) ->
            m.Operation="shared-leg-move-hinge-site"
            && m.Changes |> List.forall (fun change ->
                change.Path.EndsWith("-upper-output/sites/corner/position") && change.After=[-20.0;0.0]))
    let altered = Realization.screen hinge.Program 72 |> unwrap
    let changed = WalkerEvolution.realize four r4.Genome hinge altered 1 |> unwrap |> List.head
    WalkerRealization.compile altered changed |> unwrap |> ignore
    Assert.Equal(4,hinge.Changes.Length)
    Assert.Equal<LegInstance list>(r4.Genome.LegFamilies.Value.Head.Instances,changed.LegFamilies.Value.Head.Instances)
    let _,_,proportions =
        catalogue |> List.find (fun (_,_,m) ->
            m.Operation="shared-leg-limb-length"
            && (m.Changes |> List.filter (fun c ->
                c.Path.EndsWith("-leg-a/sites/tip/position") && c.After=[100.0;0.0])).Length=4)
    Assert.Equal(8,proportions.Changes.Length)
    let resized = Realization.screen proportions.Program 72 |> unwrap
    let rebuilt = WalkerEvolution.realize four r4.Genome proportions resized 1 |> unwrap |> List.head
    let resizedRobot = WalkerRealization.compile resized rebuilt |> unwrap
    Assert.Equal<LegInstance list>(r4.Genome.LegFamilies.Value.Head.Instances,rebuilt.LegFamilies.Value.Head.Instances)
    Assert.Equal(four.Program.Motor,resized.Program.Motor)
    Assert.Equal<MotionTransmission list>(four.Program.Transmissions,resized.Program.Transmissions)
    Assert.Equal<MotionFoot list>(four.Program.Feet,resized.Program.Feet)
    let ground = four.Program.Bodies |> List.find _.Grounded
    Assert.Equal(ground,resized.Program.Bodies |> List.find (fun b -> b.Id=ground.Id))
    Assert.Equal(r4.Genome.Bodies |> List.find (fun b -> b.Body=ground.Id),
                 rebuilt.Bodies |> List.find (fun b -> b.Body=ground.Id))
    let upperLinks (g: WalkerGenome) =
        g.Bodies |> List.filter (fun b -> b.Body.EndsWith("-leg-a") || b.Body.EndsWith("-upper-parallel"))
        |> List.collect _.Parts
    let beforeLinks,afterLinks = upperLinks r4.Genome,upperLinks rebuilt
    Assert.Equal(8,afterLinks.Length)
    Assert.All(afterLinks,fun part ->
        let before = beforeLinks |> List.find (fun p -> p.Id=part.Id)
        Assert.NotEqual<string>(before.Part,part.Part))
    Assert.Equal(4,resizedRobot.Assembly.Parts |> List.filter (fun p -> p.Part="32526.dat") |> List.length)
    Assert.All(resized.Program.Feet,fun foot ->
        Assert.All(resized.Frames,fun frame ->
            Assert.InRange(abs frame.Poses[foot.Port.Body].Angle,0.0,1e-6)))
    let _,_,lower =
        catalogue |> List.find (fun (_,_,m) ->
            m.Operation="shared-leg-limb-length"
            && (m.Changes |> List.filter (fun c ->
                c.Path.EndsWith("-leg-b/sites/bottom/position") && c.After=[-100.0;0.0])).Length=4)
    let shortened = Realization.screen lower.Program 72 |> unwrap
    let lowerFit = WalkerEvolution.realize four r4.Genome lower shortened 1 |> unwrap |> List.head
    WalkerRealization.compile shortened lowerFit |> unwrap |> ignore
    Assert.Equal<LegInstance list>(r4.Genome.LegFamilies.Value.Head.Instances,lowerFit.LegFamilies.Value.Head.Instances)
    Assert.Single(lowerFit.LegFamilies.Value.Head.Instances
                  |> List.map (WalkerLegFamilies.shapeSignatureAt shortened.Frames.Head.Poses lowerFit)
                  |> List.distinct) |> ignore
    let gait = catalogue |> List.filter (fun (_,_,m) -> m.Operation="gait-half-cycle")
    Assert.NotEmpty gait
    let _,_,mutation = gait.Head
    let rephased = Realization.screen mutation.Program 72 |> unwrap
    let choices = WalkerEvolution.realize four r4.Genome mutation rephased 1 |> unwrap
    let phased = WalkerRealization.compile rephased choices.Head |> unwrap
    Assert.Equal(4,phased.Genome.LegFamilies.Value.Head.Instances.Length)
    Assert.Equal(1,phased.Genome.LegFamilies.Value.Head.Instances |> List.filter (fun i -> abs(i.Phase-Math.PI)<1e-6) |> List.length)
