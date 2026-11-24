#r "../src/LegoWalkers.Core/bin/Debug/net10.0/LegoWalkers.Core.dll"

open System
open System.IO
open System.Text.Json.Nodes
open LegoWalkers

let source,output =
    match fsi.CommandLineArgs |> Array.skip 1 |> Array.toList with
    | ["--program"; source; "--output"; output] -> Path.GetFullPath source,Path.GetFullPath output
    | _ -> invalidArg "arguments" "Expected --program SINGLE_LEG_PROGRAM_JSON --output EMPTY_DIRECTORY."
if Directory.Exists output && not (Directory.EnumerateFileSystemEntries output |> Seq.isEmpty) then
    invalidArg "output" "Fixture output must be empty."
let program = MotionExport.readProgram source
let s = Realization.screen program 144 |> Result.defaultWith (Realization.json >> invalidOp)
let expected = Set.ofList ["chassis";"crank";"lower-arm";"upper-arm";"upright-l";"driven-leg";"parallel-leg";"shoe"]
if (program.Bodies |> List.map _.Id |> Set.ofList) <> expected || program.Hinges.Length <> 10 then
    invalidArg "program" "Expected the confirmed single-leg reference graph."
let pose = s.Frames.Head.Poses
let point body site =
    let b = program.Bodies |> List.find (fun b -> b.Id = body)
    Motion.transform pose[body] (b.Sites |> List.find (fun s -> s.Id = site)).Position
let near a b = Vec3.distance a b < 1e-6
let requirePoint body site expected =
    if not (near (point body site) expected) then invalidArg "program" $"Unexpected reference dimensions at {body}/{site}."
for body,site,p in [
    "chassis","lower",Vec3.create -80 0 0
    "chassis","upper",Vec3.create -80 -60 0
    "chassis","drive",Vec3.create -20 80 0
    "upright-l","corner",Vec3.create 0 0 0
    "upright-l","top",Vec3.create 0 -60 0
    "upright-l","toe-link",Vec3.create 40 0 0
    "driven-leg","drive",Vec3.create 0 80 0
    "driven-leg","bottom",Vec3.create 0 200 0
    "parallel-leg","top",Vec3.create 40 0 0
    "shoe","left",Vec3.create 0 200 0
    "shoe","right",Vec3.create 40 200 0
] do requirePoint body site p

let parts,joints = ResizeArray<PartInstance>(),ResizeArray<Joint>()
let add id part color p rotation owner =
    parts.Add { Id=id; Part=part; Color=color; Position=p; Rotation=rotation; Body=owner }
let structural id part color p rotation body = add id part color p rotation (Some body)
let depth z (p: Vec3) = { p with Z=z }
let beam id part color z body a b =
    structural id part color (Vec3.scale 0.5 (Vec3.add a b) |> depth z) (Rotation.beam (Vec3.subtract b a)) body
beam "lower-arm-part" "32316.dat" 4 0 "lower-arm" (point "lower-arm" "root") (point "lower-arm" "tip")
beam "upper-arm-part" "32316.dat" 14 20 "upper-arm" (point "upper-arm" "root") (point "upper-arm" "tip")
beam "driven-leg-part" "32525.dat" 72 20 "driven-leg" (point "driven-leg" "top") (point "driven-leg" "bottom")
beam "parallel-leg-part" "32525.dat" 14 20 "parallel-leg" (point "parallel-leg" "top") (point "parallel-leg" "bottom")
let lPosition,lRotation =
    ["corner";"top";"toe-link"] |> List.map (point "upright-l" >> depth 40)
    |> Catalog.fitPlanarPart "32526.dat" (Vec3.create 0 0 1)
    |> List.tryHead |> Option.defaultWith (fun () -> invalidOp "Native catalogue cannot fit the reference L sockets.")
structural "upright-l-part" "32526.dat" 14 lPosition lRotation "upright-l"
structural "crank-part" "60483.dat" 0 (point "crank" "root") (Rotation.beam (Vec3.create 1 0 0)) "crank"
structural "shoe-part" "3701.dat" 14 (Vec3.create 20 190 40) Rotation.identity "shoe"
structural "frame-rail" "3703.dat" 0 (Vec3.create -20 70 -40) Rotation.identity "chassis"
structural "frame-upright" "32525.dat" 71 (Vec3.create -80 -20 -20) (Rotation.beam (Vec3.create 0 1 0)) "chassis"
structural "input-motor" "58121.dat" 7 (Vec3.create -20 80 -70)
    { Rotation.identity with X=Vec3.create -1 0 0; Z=Vec3.create 0 0 -1 } "chassis"
structural "motor-mount" "32523.dat" 0 (Vec3.create -20 80 -60) (Rotation.beam (Vec3.create 1 0 0)) "chassis"
let shaft = Rotation.shaft 0
add "frame-mount-pin" "4459.dat" 1 (Vec3.create -80 80 -30) shaft None
for i,x in [-40.0;0.0] |> List.indexed do
    add $"motor-mount-pin-{i}" "6558.dat" 1 (Vec3.create x 80 -60) shaft None
let pivots = [
    "bearing",["bearing"],["chassis";"crank"],"44294.dat",Vec3.create -20 80 -59.5
    "crank-mid-leg",["crank-mid-leg"],["crank";"driven-leg"],"3673.dat",Vec3.create 0 80 10
    "lower-bearing",["lower-bearing"],["chassis";"lower-arm"],"3673.dat",Vec3.create -80 0 -10
    "shared-corner-a",["shared-corner-a";"shared-corner-b"],["lower-arm";"driven-leg";"upright-l"],"32556.dat",Vec3.create 0 0 20
    "upper-bearing",["upper-bearing"],["chassis";"upper-arm"],"32556.dat",Vec3.create -80 -60 0
    "upper-upright",["upper-upright"],["upper-arm";"upright-l"],"3673.dat",Vec3.create 0 -60 30
    "parallel-top",["parallel-top"],["upright-l";"parallel-leg"],"3673.dat",Vec3.create 40 0 30
    "parallel-shoe",["parallel-shoe"],["parallel-leg";"shoe"],"3673.dat",Vec3.create 40 200 30
    "driven-shoe",["driven-shoe"],["driven-leg";"shoe"],"3673.dat",Vec3.create 0 200 30
]
for id,_,members,part,p in pivots do
    add ("pivot-"+id) part 7 p shaft None
    for i,(a,b) in members |> List.pairwise |> List.indexed do
        joints.Add { Id = $"joint-{id}-{i}"; BodyA=a; BodyB=b; Position=p
                     Axis=Vec3.create 0 0 1; Connector="pivot-"+id }

let localPorts part = (Catalog.get part).Ports
let touches (pin: PartInstance) (part: PartInstance) =
    let low,high = Catalog.shaftRange pin.Part
    localPorts part.Part |> List.exists (fun port ->
        let world = Vec3.add part.Position (Rotation.apply part.Rotation port.Position)
        let delta = Vec3.subtract world pin.Position
        let axis = Rotation.apply part.Rotation port.Axis
        let axial = Vec3.dot delta pin.Rotation.X
        abs (Vec3.dot axis pin.Rotation.X) > 1.0-1e-8
        && Vec3.length (Vec3.cross delta pin.Rotation.X) < 1e-7 && axial >= low-1e-7 && axial <= high+1e-7)
let structuralParts = parts |> Seq.filter _.Body.IsSome |> Seq.toList
for pin in parts |> Seq.filter _.Body.IsNone do
    let contacted = structuralParts |> List.filter (touches pin)
    let low,high = Catalog.shaftRange pin.Part
    let capacity = max 2 (int (Math.Round((high-low)/20.0)))
    if contacted.Length > capacity then invalidOp $"Pin contact capacity exceeded at {pin.Id}."
    let expectedBodies =
        match pivots |> List.tryFind (fun (id,_,_,_,_) -> "pivot-"+id = pin.Id) with
        | Some (_,_,members,_,_) -> Set.ofList members
        | None -> Set.singleton "chassis"
    if (contacted |> List.map (_.Body >> Option.get) |> Set.ofList) <> expectedBodies then
        invalidOp $"Incorrect analytical pin engagement at {pin.Id}: {contacted |> List.map _.Id}"
for pin in parts |> Seq.filter _.Body.IsNone do
    for other in parts |> Seq.filter (fun p -> p.Body.IsNone && p.Id > pin.Id) do
        if Catalog.shaftsOverlap pin.Part pin.Position pin.Rotation other.Part other.Position other.Rotation then
            invalidOp $"Connector shafts overlap: {pin.Id}, {other.Id}."
let footPoints = s.Frames |> List.map (fun f -> (Motion.footPositions s.Graph f)["foot"] |> depth 40)
let assembly = {
    Kind=Leg; Genome=Genome.defaults
    Parts=parts |> Seq.sortBy (fun p -> if p.Body=Some "chassis" then 0 else 1) |> Seq.toList
    Joints=List.ofSeq joints
    Gears=[]; Drives=[{ MotorPart="input-motor"; Joint="joint-bearing-0"; Speed=3; MaxTorque=0.4 }]
    Feet=[{ Id="foot"; Body="shoe"; Position=footPoints.Head; Part="shoe-part" }]
    FootPaths=[{ Id="foot"; Points=footPoints
                 Stride=(footPoints |> List.maxBy _.X).X-(footPoints |> List.minBy _.X).X
                 Lift=(footPoints |> List.maxBy _.Y).Y-(footPoints |> List.minBy _.Y).Y }]
    Chassis="chassis"
    Warnings=["Hand-authored single-leg construction fixture, not a walker or an automatically generated recipe."
              "Explicit fixture placements retained as the validated dynamics control; native construction proposals are checked separately."
              "Direct motor on crank is a test-fixture input, not the photographed robot's reduction train."
              "Socket engagement is checked; axial retention, load capacity, complete swept clearance and walking are not certified."]
}
for p in parts do
    if not (Vec3.finite p.Position && Rotation.valid p.Rotation) then invalidOp $"Invalid placement {p.Id}"
let genome : RealizationGenome = {
    SchemaVersion=1; RecipeVersion=Realization.recipeVersion; MechanismId=s.MechanismId; Side=1
    Bodies=[for body in program.Bodies do
                let part = structuralParts |> List.find (fun p -> p.Body=Some body.Id)
                yield { Body=body.Id; Part=part.Part; Layer=int ((part.Position.Z+40.0)/20.0); Offset=0 }]
    Connections=[for id,_,_,part,p in pivots -> { Group=id; Part=part; Center=p.Z }]
}
// V2 is the existing interchange contract; the full hand-authored recipe is frozen in fixture.json.
let manifest = JsonNode.Parse(Export.manifest "model.ldr" assembly)
manifest["grammarVersion"] <- JsonValue.Create Realization.version
manifest["genome"] <- JsonNode.Parse(Realization.json genome)
for foot in manifest["feet"].AsArray() do foot["logicalFoot"] <- JsonValue.Create "foot"
manifest["motion"] <- JsonNode.Parse(Realization.json {|
    mechanismId=s.MechanismId; genotypeId=Export.sha256 (Realization.json genome + Export.ldraw assembly)
    assemblyId=Export.assemblyId assembly; gateVersion=Realization.gateVersion; cycleSteps=s.Frames.Length-1
    footCount=1; logicalFeet=[{| id="foot";body="shoe";regions=["foot"] |}]
    sharedPivots=[for id,hinges,members,_,_ in pivots -> {|id=id;abstractHinges=hinges;bodies=members|}]
|})
Directory.CreateDirectory output |> ignore
File.WriteAllText(Path.Combine(output,"model.ldr"),Export.ldraw assembly)
File.WriteAllText(Path.Combine(output,"manifest.json"),manifest.ToJsonString()+"\n")
File.WriteAllText(Path.Combine(output,"program.json"),MotionExport.programJson program)
File.WriteAllText(Path.Combine(output,"cycle.svg"),MotionExport.cycleSvg program s.Graph s.Frames)
File.WriteAllText(Path.Combine(output,"fixture.json"),Realization.json {|
    schemaVersion=1; recipe="photo-double-parallelogram-leg-fixture-v4"
    lBranchOrientation="Away from chassis/red arm; driven leg, upper linkage and crank unchanged."
    driveAxlePlacement="7L axle recessed so the actual mesh ends flush at the crank's Z=10 face, outside the driven leg's swept depth."
    classification="hand-authored-single-leg-fixture-not-native-search-candidate"
    inputProgramSha256=Export.sha256 (File.ReadAllText source)
    mechanismId=s.MechanismId; assemblyId=Export.assemblyId assembly
    input="Direct-drive test fixture; photographed upstream reduction is not reconstructed."
    interchange="Motion-v2 manifest for exact adapter checks; this control layout is not a compiler proposal. Full placements below are authoritative."
    parts=assembly.Parts; joints=assembly.Joints; sampledFrames=s.Frames.Length
    maxKinematicResidualLdu=s.Frames |> List.map _.MaxResidual |> List.max
    nativeCandidate=false; dynamicTrials=0
|}+"\n")
printfn "%d real parts, %d bodies, %d hinge constraints, %d physical pivots; zero dynamics."
    parts.Count expected.Count joints.Count pivots.Length
