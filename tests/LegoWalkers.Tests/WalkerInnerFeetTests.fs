module LegoWalkers.WalkerInnerFeetTests

open System
open System.IO
open LegoWalkers
open Xunit

let private unwrap result = Result.defaultWith (Realization.json >> failwith) result
let private repaired () =
    let program,genome = Walker1Seeds.walker 2
    let screened = Realization.screen program 72 |> unwrap
    let baseline = WalkerRealization.compile screened genome |> unwrap
    let candidate = WalkerInnerFeet.add screened genome |> unwrap
    screened,baseline,WalkerRealization.compile screened candidate |> unwrap

[<Fact>]
let ``inner soles retain the mechanism and use covered 3L smooth ankle pins`` () =
    let screened,before,after = repaired ()
    Assert.Equal(before.Assembly.Parts.Length+2,after.Assembly.Parts.Length)
    Assert.Equal(before.Genome.Bodies.Length,after.Genome.Bodies.Length)
    Assert.Equal<WalkerPlacement list>(before.Genome.Mounts,after.Genome.Mounts)
    Assert.Equal<GearMesh list>(before.Assembly.Gears,after.Assembly.Gears)
    Assert.Equal<Drive list>(before.Assembly.Drives,after.Assembly.Drives)
    Assert.Equal(2,Motion.footCount screened.Program)
    Assert.Equal(16,after.Assembly.Feet.Length)
    for choice in after.Genome.Feet do
        let inner = choice.AdditionalContacts |> Option.get |> List.exactlyOne
        Assert.Equal(40.0,abs(inner.Depth-choice.Depth),6)
        let body = after.Genome.Bodies |> List.find (fun b -> b.Parts |> List.exists (fun p -> p.Id=choice.Part))
        Assert.Equal(2,body.Parts.Length)
        let innerBody = after.Genome.Bodies |> List.find (fun b -> b.Parts |> List.exists (fun p -> p.Id=inner.Part))
        Assert.Equal<string>(body.Body,innerBody.Body)
        let joints = after.Genome.Connections |> List.filter (fun p ->
            p.Hinges |> List.exists (fun id ->
                screened.Program.Hinges |> List.exists (fun h -> h.Id=id && (h.A.Body=body.Body || h.B.Body=body.Body))))
        Assert.Equal(2,joints.Length)
        for joint in joints do
            Assert.Equal("32556.dat",joint.Placement.Part)
            Assert.Single(joint.Hinges) |> ignore
            let pin = joint.Placement
            let members =
                screened.Program.Hinges |> List.filter (fun h -> List.contains h.Id joint.Hinges)
                |> List.collect (fun h -> [h.A.Body;h.B.Body]) |> Set.ofList
            let contacts =
                after.Genome.Bodies |> List.filter (fun b -> members.Contains b.Body) |> List.collect _.Parts
                |> List.filter (fun part ->
                    (Catalog.get part.Part).Ports |> List.exists (fun port ->
                        let point = Vec3.add part.Position (Rotation.apply part.Rotation port.Position)
                        abs(point.X-pin.Position.X)<1e-6 && abs(point.Y-pin.Position.Y)<1e-6))
            Assert.Equal(3,contacts.Length)
            let walls = contacts |> List.collect (fun part ->
                let center = Vec3.dot (Vec3.subtract part.Position pin.Position) pin.Rotation.X
                let half = Catalog.socketHalfThickness part.Part
                [center-half;center+half])
            let low,high = Catalog.shaftRange pin.Part
            Assert.True(List.min walls>=low-1e-6 && List.max walls<=high+1e-6)
            let shortLow,shortHigh = Catalog.shaftRange "3673.dat"
            Assert.True(List.max walls-List.min walls>shortHigh-shortLow)
    Assert.Equal<string list>([],Grammar.validateAssembly after.Assembly)

[<Fact>]
let ``paired foot choices round trip without adding logical feet or reclassifying leg beams`` () =
    let screened,_,robot = repaired ()
    let directory = Path.Combine(Path.GetTempPath(),Guid.NewGuid().ToString("N"))
    try
        WalkerRealization.write directory screened robot |> ignore
        let decoded = WalkerRealization.readGenome (Path.Combine(directory,"realization.json"))
        Assert.Equal(robot.GenotypeId,(WalkerRealization.compile screened decoded |> unwrap).GenotypeId)
        Assert.Equal(2,robot.LogicalFeet |> Map.values |> Set.ofSeq |> Set.count)
        Assert.All(robot.Assembly.Feet,fun f ->
            Assert.Equal("3701.dat",(robot.Assembly.Parts |> List.find (fun p -> p.Id=f.Part)).Part))
        Assert.True((WalkerInnerFeet.add screened decoded).IsError)
    finally
        if Directory.Exists directory then Directory.Delete(directory,true)

[<Theory>]
[<InlineData("missing")>]
[<InlineData("depth")>]
let ``inner contact declarations reject missing bricks and incorrect layers`` change =
    let screened,_,robot = repaired ()
    let edit (contact: WalkerFootContact) =
        if change="missing" then {contact with Part="missing-inner-sole"}
        else {contact with Depth=contact.Depth+20.0}
    let feet = robot.Genome.Feet |> List.mapi (fun index choice ->
        if index<>0 then choice else
            {choice with AdditionalContacts=choice.AdditionalContacts |> Option.map (List.map edit)})
    Assert.True((WalkerRealization.compile screened {robot.Genome with Feet=feet}).IsError)

[<Theory>]
[<InlineData(1)>]
[<InlineData(2)>]
let ``plate soles are real stud mounted parts and retain the original mechanism`` layers =
    let screened,_,before = repaired ()
    let genome = WalkerFootPlates.add screened before.Genome layers |> unwrap
    let after = WalkerRealization.compile screened genome |> unwrap
    Assert.Equal(before.Assembly.Parts.Length+4*layers,after.Assembly.Parts.Length)
    Assert.Equal(before.Genome.Bodies.Length,after.Genome.Bodies.Length)
    Assert.Equal<WalkerPivotChoice list>(before.Genome.Connections,after.Genome.Connections)
    Assert.Equal<GearMesh list>(before.Assembly.Gears,after.Assembly.Gears)
    Assert.Equal<Drive list>(before.Assembly.Drives,after.Assembly.Drives)
    Assert.Equal(before.Assembly.Feet.Length,after.Assembly.Feet.Length)
    for foot in after.Assembly.Feet do
        let previous = before.Assembly.Feet |> List.find (fun f -> f.Id=foot.Id)
        let original = before.Assembly.Parts |> List.find (fun p -> p.Id=previous.Part)
        let plate = after.Assembly.Parts |> List.find (fun p -> p.Id=foot.Part)
        Assert.Equal("3710.dat",plate.Part)
        Assert.Equal(0.0,Vec3.distance plate.Position
            (Vec3.add original.Position (Vec3.scale (24.0+8.0*float(layers-1)) original.Rotation.Y)),6)
        Assert.Equal(0.0,Vec3.distance foot.Position
            (Vec3.add previous.Position (Vec3.scale (8.0*float layers) original.Rotation.Y)),6)
    Assert.True((WalkerFootPlates.add screened after.Genome 1).IsError)

[<Theory>]
[<InlineData("missing")>]
[<InlineData("offset")>]
[<InlineData("three-layers")>]
let ``unattached or excessive plate stacks are rejected`` change =
    let screened,_,before = repaired ()
    if change="three-layers" then
        Assert.True((WalkerFootPlates.add screened before.Genome 3).IsError)
    else
        let genome = WalkerFootPlates.add screened before.Genome 1 |> unwrap
        let primary = genome.Feet.Head.Part
        let bodies = genome.Bodies |> List.map (fun body ->
            let parts = body.Parts |> List.choose (fun p ->
                if change="missing" && p.Id=before.Genome.Feet.Head.Part then None
                elif change="offset" && p.Id=primary then
                    Some {p with Position=Vec3.add p.Position (Vec3.scale 20.0 p.Rotation.X)}
                else Some p)
            {body with Parts=parts})
        Assert.True((WalkerRealization.compile screened {genome with Bodies=bodies}).IsError)

[<Fact>]
let ``parallel soles work when the original shoe is inside the ankle links`` () =
    let screened,_,paired = repaired ()
    let originalParts = paired.Genome.Feet |> List.map _.Part |> Set.ofList
    let feet = paired.Genome.Feet |> List.map (fun foot ->
        let inner = foot.AdditionalContacts.Value |> List.exactlyOne
        {foot with Part=inner.Part;Depth=inner.Depth;AdditionalContacts=None})
    let bodies = paired.Genome.Bodies |> List.map (fun body ->
        {body with Parts=body.Parts |> List.filter (fun part -> not(originalParts.Contains part.Id))})
    let connections = paired.Genome.Connections |> List.map (fun pivot ->
        if pivot.Placement.Part<>"32556.dat" then pivot else
            let matching = feet |> List.tryFind (fun foot ->
                let body = bodies |> List.find (fun b -> b.Parts |> List.exists (fun p -> p.Id=foot.Part))
                pivot.Hinges |> List.exists (fun id ->
                    screened.Program.Hinges |> List.exists (fun h -> h.Id=id && (h.A.Body=body.Body || h.B.Body=body.Body))))
            match matching with
            | None -> pivot
            | Some foot ->
                {pivot with Placement={pivot.Placement with Part="3673.dat";
                                                             Position={pivot.Placement.Position with Z=(pivot.Placement.Position.Z+foot.Depth)/2.0}}})
    let inside = {paired.Genome with Bodies=bodies;Connections=connections;Feet=feet}
    WalkerRealization.compile screened inside |> unwrap |> ignore
    let upgraded,gates = WalkerFootPlates.prepare screened inside |> unwrap
    Assert.All(gates,fun gate -> Assert.Equal("pass",gate.Outcome))
    for foot in upgraded.Feet do
        Assert.Single(foot.AdditionalContacts.Value) |> ignore
        Assert.Contains("-outer-plate-1",foot.AdditionalContacts.Value.Head.Part)
    Assert.Equal(2,Motion.footCount screened.Program)

[<Fact>]
let ``automatic paired plate feet are idempotent and do not change the drive or gait`` () =
    let program,genome = Walker1Seeds.walker 2
    let screened = Realization.screen program 72 |> unwrap
    let original = WalkerRealization.compile screened genome |> unwrap
    let prepared,gates = WalkerFootPlates.prepare screened genome |> unwrap
    let after = WalkerRealization.compile screened prepared |> unwrap
    let repeated,_ = WalkerFootPlates.prepare screened prepared |> unwrap
    Assert.Equal<WalkerGenome>(prepared,repeated)
    Assert.All(gates,fun gate -> Assert.Equal("pass",gate.Outcome))
    Assert.Equal(original.Assembly.Parts.Length+6,after.Assembly.Parts.Length)
    Assert.Equal<Drive list>(original.Assembly.Drives,after.Assembly.Drives)
    Assert.Equal<GearMesh list>(original.Assembly.Gears,after.Assembly.Gears)
    Assert.All(after.Assembly.Feet,fun foot ->
        Assert.Equal("3710.dat",(after.Assembly.Parts |> List.find (fun p -> p.Id=foot.Part)).Part))

[<Fact>]
let ``unsupported toe forms are retained with explicit construction diagnostics`` () =
    let program,genome = WalkerSeeds.fourBar 2
    let screened = Realization.screen program 72 |> unwrap
    let prepared,gates = WalkerFootPlates.prepare screened genome |> unwrap
    Assert.Equal<WalkerGenome>(genome,prepared)
    Assert.Contains(gates,fun gate -> gate.Outcome="warning" && not gate.Diagnostics.IsEmpty)

[<Theory>]
[<InlineData(1)>]
[<InlineData(2)>]
let ``single plated ankles are paired with only lower plates permitted as contacts`` layers =
    let program,genome = Walker1Seeds.walker 2
    let screened = Realization.screen program 72 |> unwrap
    let single = WalkerFootPlates.add screened genome layers |> unwrap
    let paired,_ = WalkerFootPlates.prepareWithLayers layers screened single |> unwrap
    let repeated,_ = WalkerFootPlates.prepareWithLayers layers screened paired |> unwrap
    Assert.Equal<WalkerGenome>(paired,repeated)
    Assert.Equal(2,paired.Feet.Length)
    for foot in paired.Feet do
        let contacts = WalkerFootChoice.contacts foot
        Assert.Equal(2,contacts.Length)
        for contact in contacts do
            Assert.EndsWith($"-plate-{layers}",contact.Part)
    let robot = WalkerRealization.compile screened paired |> unwrap
    Assert.Equal<string list>([],Grammar.validateAssembly robot.Assembly)

[<Fact>]
let ``paired one layer feet extend to two layers without altering their ankle pins`` () =
    let program,genome = Walker1Seeds.walker 2
    let screened = Realization.screen program 72 |> unwrap
    let first,_ = WalkerFootPlates.prepare screened genome |> unwrap
    let second,_ = WalkerFootPlates.prepareWithLayers 2 screened first |> unwrap
    Assert.Equal<WalkerPivotChoice list>(first.Connections,second.Connections)
    Assert.Equal(first.Bodies |> List.sumBy (fun b -> b.Parts.Length) |> (+) 4,
                 second.Bodies |> List.sumBy (fun b -> b.Parts.Length))
    Assert.All(second.Feet |> List.collect WalkerFootChoice.contacts,fun contact ->
        Assert.EndsWith("-plate-2",contact.Part))
