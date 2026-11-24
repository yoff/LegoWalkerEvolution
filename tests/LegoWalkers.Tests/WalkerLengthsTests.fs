module LegoWalkers.WalkerLengthsTests

open LegoWalkers
open Xunit

let private unwrap result = Result.defaultWith (Realization.json >> failwith) result

let private seed () =
    let program,genome = WalkerSeeds.fourBar 2
    let screened = Realization.screen program 72 |> unwrap
    screened,(WalkerRealization.compile screened genome |> unwrap).Genome

let private mutation (program,changes) : WalkerMutation =
    {Program=program;Operation="limb-length"
     Changes=changes |> List.map (fun (path,before,after) -> {Path=path;Before=before;After=after})
     RotatedPart=None;Dyad=None;RemovedBodies=[];FootReplacement=None;Physical=None;Components=[];FamilyUpdate=None}

let private beamBottom (p: WalkerPlacement) =
    let holeBottom =
        (Catalog.get p.Part).Ports |> List.map (fun socket ->
            (Vec3.add p.Position (Rotation.apply p.Rotation socket.Position)).Y) |> List.max
    holeBottom+9.0*(abs p.Rotation.X.Y+abs p.Rotation.Z.Y)+10.0*abs p.Rotation.Y.Y

let private soleFloor (g: WalkerGenome) =
    let ids = g.Feet |> List.map _.Part
    g.Bodies |> List.collect _.Parts |> List.filter (fun p -> List.contains p.Id ids)
    |> List.map (fun p ->
        Assert.Equal("3701.dat",p.Part)
        p.Position.Y+10.0*p.Rotation.Y.Y+40.0*abs p.Rotation.X.Y+14.0*abs p.Rotation.Y.Y+10.0*abs p.Rotation.Z.Y)
    |> List.max

[<Fact>]
let ``length proposals are deterministic motion edits preserving anchors and controls`` () =
    let parent,genome = seed ()
    let proposals = WalkerLengths.proposals parent genome
    Assert.NotEmpty proposals
    Assert.Equal<(MotionProgram * (string * float list * float list) list) list>(
        proposals,WalkerLengths.proposals parent genome)
    let protectedIds =
        parent.Program.Bodies |> List.filter _.Grounded |> List.map _.Id
        |> Set.ofList |> Set.add parent.Program.Motor.Body
        |> Set.union (parent.Program.Transmissions |> List.collect (fun t -> [t.Input;t.Output]) |> Set.ofList)
    for program,changes in proposals do
        Assert.NotEmpty changes
        Assert.Equal(parent.Program.Motor,program.Motor)
        Assert.Equal<MotionTransmission list>(parent.Program.Transmissions,program.Transmissions)
        Assert.Equal<MotionFoot list>(parent.Program.Feet,program.Feet)
        for body in parent.Program.Bodies do
            let after = program.Bodies |> List.find (fun b -> b.Id=body.Id)
            Assert.Equal(body.Initial,after.Initial)
            if protectedIds.Contains body.Id then Assert.Equal(body,after)

[<Fact>]
let ``a passive span can outgrow its original beam and receive a longer stock part`` () =
    let source,original = seed ()
    let motionBodies =
        source.Program.Bodies |> List.map (fun b ->
            if b.Id.EndsWith("-rocker") then {b with Sites=b.Sites |> List.filter (fun s -> s.Id<>"end")} else b)
    let program = {source.Program with Bodies=motionBodies}
    let parent = Realization.screen program 72 |> unwrap
    let bodies =
        original.Bodies |> List.map (fun body ->
            if not (body.Body.EndsWith("-rocker")) then body else
                let part = body.Parts.Head
                let points =
                    (parent.Program.Bodies |> List.find (fun b -> b.Id=body.Body)).Sites
                    |> List.map (fun s ->
                        let point = Motion.transform parent.Frames.Head.Poses[body.Body] s.Position
                        {point with Z=part.Position.Z})
                let position,rotation =
                    Catalog.fitPlanarPart "32316.dat" (Vec3.create 0 0 1) points |> List.head
                {body with Parts=[{part with Part="32316.dat";Position=position;Rotation=rotation}]})
    let genome =
        WalkerRealization.compile parent {original with MechanismId=parent.MechanismId;Bodies=bodies}
        |> unwrap |> _.Genome
    let candidates =
        WalkerLengths.proposals parent genome
        |> List.filter (fun (_,changes) ->
            changes |> List.exists (fun (path,before,after) ->
                path.EndsWith("-rocker/sites/joint/position") && after.Head>before.Head))
        |> List.map mutation
    Assert.NotEmpty candidates
    let outcomes =
        candidates |> List.map (fun edit ->
            Realization.screen edit.Program 72 |> Result.bind (fun child ->
                WalkerEvolution.realize parent genome edit child 1
                |> Result.bind (fun choices -> WalkerRealization.compile child choices.Head)))
    let robots = outcomes |> List.choose (function Ok robot -> Some robot | _ -> None)
    let errors = outcomes |> List.choose (function Error error -> Some (Realization.json error) | _ -> None)
    Assert.True(not robots.IsEmpty,System.String.Join("\n",errors))
    let rebuilt = robots.Head.Genome
    let originals = genome.Bodies |> List.collect _.Parts
    let replacements =
        rebuilt.Bodies |> List.collect _.Parts |> List.filter (fun part ->
            let old = originals |> List.find (fun p -> p.Id=part.Id)
            old.Part<>part.Part)
    Assert.NotEmpty replacements
    Assert.Equal(genome.Speed,rebuilt.Speed)
    Assert.Equal(genome.MaxTorque,rebuilt.MaxTorque)
    Assert.Equal(genome.MotorPart,rebuilt.MotorPart)
    Assert.Equal<WalkerFootChoice list>(genome.Feet,rebuilt.Feet)

[<Fact>]
let ``compound or rigidly mounted passive bodies are not silently refitted`` () =
    let parent,genome = seed ()
    let bodies =
        genome.Bodies |> List.map (fun b ->
            if b.Body.EndsWith("-rocker") then
                {b with Parts=b.Parts @ [{b.Parts.Head with Id=b.Parts.Head.Id+"-brace"}]}
            else b)
    let compound = {genome with Bodies=bodies}
    for _,changes in WalkerLengths.proposals parent compound do
        Assert.DoesNotContain(changes,fun (path,_,_) -> path.Contains("-rocker/sites/"))
    let body = genome.Bodies |> List.find (fun b -> b.Body.EndsWith("-rocker"))
    let part = body.Parts.Head
    let socket = (Catalog.get part.Part).Ports.Head
    let mount = {
        Id="rigid-brace";Part="4459.dat";Color=0
        Position=Vec3.add part.Position (Rotation.apply part.Rotation socket.Position)
        Rotation={X=Vec3.create 0 0 1;Y=Vec3.create 0 1 0;Z=Vec3.create -1 0 0}
    }
    let mounted = {genome with Mounts=mount::genome.Mounts}
    for _,changes in WalkerLengths.proposals parent mounted do
        Assert.DoesNotContain(changes,fun (path,_,_) -> path.StartsWith($"bodies/{body.Body}/"))

[<Fact>]
let ``lower L spans change with their parallel link and reject unavailable stock lengths`` () =
    let program,genome = Walker1Seeds.walker 2
    let source = Realization.screen program 72 |> unwrap
    let replacement = WalkerLevelFeet.edits source genome |> List.head
    let parent = Realization.screen replacement.Program 72 |> unwrap
    let physical = (WalkerRealization.compile parent replacement.Genome |> unwrap).Genome
    let proposals = WalkerLengths.proposals parent physical
    for bottom,span,available in [-100.0,180.0,true;-140.0,220.0,false] do
        let proposal =
            proposals |> List.find (fun (_,changes) ->
                changes |> List.exists (fun (path,_,after) ->
                    path.EndsWith("-leg-b/sites/bottom/position") && after=[bottom;0.0]))
        let edit = mutation proposal
        Assert.Equal(2,edit.Changes.Length)
        Assert.Contains(edit.Changes,fun c ->
            c.Path.EndsWith("-sole-parallel/sites/tip/position") && abs(c.After.Head-span)<1e-6)
        let child = Realization.screen edit.Program 72 |> unwrap
        match WalkerEvolution.realize parent physical edit child 1 with
        | Ok choices ->
            Assert.True(available)
            let robot = WalkerRealization.compile child choices.Head |> unwrap
            Assert.Equal<WalkerFootChoice list>(physical.Feet,robot.Genome.Feet)
            Assert.Equal(parent.Program.Motor,child.Program.Motor)
        | Error error ->
            Assert.False(available,Realization.json error)
            Assert.Contains(error.Diagnostics,fun message -> message.Contains("No supported stock beam"))

[<Fact>]
let ``shortened lower links avoid inherited downward overhang and old fits can be repaired`` () =
    let program,genome = Walker1Seeds.walker 2
    let original = Realization.screen program 72 |> unwrap
    let first = WalkerLevelFeet.edits original genome |> List.head
    let one = Realization.screen first.Program 72 |> unwrap
    let changed = one.Program.Feet |> List.find (fun f -> f.Port.Body.EndsWith("-sole-output")) |> _.Id
    let second =
        WalkerLevelFeet.edits one first.Genome
        |> List.find (fun e -> e.Changes |> List.exists (fun (path,_,_) ->
            path.EndsWith("/level-loop") && path<> $"feet/{changed}/level-loop"))
    let parent = Realization.screen second.Program 72 |> unwrap
    let shared = WalkerLegFamilies.adopt "L-legs" (parent.Program.Feet |> List.map _.Id) parent.Program second.Genome
    let physical = (WalkerRealization.compile parent shared |> unwrap).Genome
    let edit =
        WalkerEvolution.editsWithPolicy WalkerBuilder.defaultPolicy parent physical
        |> List.find (fun (_,_,m) ->
            m.Operation="shared-leg-limb-length"
            && (m.Changes |> List.filter (fun c ->
                c.Path.EndsWith("-leg-b/sites/bottom/position") && c.After=[-100.0;0.0])).Length=2)
        |> fun (_,_,m) -> m
    let child = Realization.screen edit.Program 72 |> unwrap
    let rebuilt = WalkerEvolution.realize parent physical edit child 1 |> unwrap |> List.head
    let changedIds = edit.Changes |> List.map (fun c -> c.Path.Split('/')[1]) |> Set.ofList
    let links = rebuilt.Bodies |> List.filter (fun b -> changedIds.Contains b.Body)
    Assert.All(links,fun body -> Assert.True(beamBottom body.Parts.Head<=soleFloor rebuilt+1e-6))
    let badBodies =
        rebuilt.Bodies |> List.map (fun body ->
            if not (changedIds.Contains body.Body) then body else
                let source = body.Parts.Head
                let points =
                    (child.Program.Bodies |> List.find (fun b -> b.Id=body.Body)).Sites
                    |> List.map (fun site ->
                        let point = Motion.transform child.Frames.Head.Poses[body.Body] site.Position
                        {point with Z=source.Position.Z})
                let worst =
                    Catalog.fitPlanarPart source.Part (Vec3.create 0 0 1) points
                    |> List.map (fun (position,rotation) -> {source with Position=position;Rotation=rotation})
                    |> List.maxBy beamBottom
                {body with Parts=[worst]})
    let bad = {rebuilt with Bodies=badBodies}
    WalkerRealization.compile child bad |> unwrap |> ignore
    Assert.Contains(bad.Bodies,fun b -> changedIds.Contains b.Body && beamBottom b.Parts.Head>soleFloor bad+1.0)
    let hints = {Kind="initial-foot-clearance";Parts=links |> List.collect _.Parts |> List.map _.Id;Bodies=[]}
    let repair = WalkerConstruction.repairs child bad hints 1 |> List.exactlyOne
    let repaired = WalkerRealization.compile child repair.Genome |> unwrap
    Assert.Equal("initial-foot-clearance-beam-fit",repair.Motif)
    Assert.True(repair.ClearanceImprovement>1.0)
    Assert.All(repaired.Genome.Bodies |> List.filter (fun b -> changedIds.Contains b.Body),fun body ->
        Assert.True(beamBottom body.Parts.Head<=soleFloor repaired.Genome+1e-6))
    Assert.Equal<WalkerFootChoice list>(bad.Feet,repaired.Genome.Feet)
    Assert.Equal<WalkerPivotChoice list>(bad.Connections,repaired.Genome.Connections)
    Assert.Equal(bad.Speed,repaired.Genome.Speed)
    Assert.Equal(bad.MaxTorque,repaired.Genome.MaxTorque)
    Assert.Empty(WalkerConstruction.repairs child repaired.Genome hints 1)
    Assert.Empty(WalkerConstruction.repairs child bad {hints with Parts=[bad.MotorPart]} 1)
