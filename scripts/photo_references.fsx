#r "../src/LegoWalkers.Core/bin/Debug/net10.0/LegoWalkers.Core.dll"

open System
open System.IO
open System.Globalization
open LegoWalkers

let unwrap result = Result.defaultWith invalidOp result
let number (value: float) = value.ToString("0.#########", CultureInfo.InvariantCulture)
let output =
    match fsi.CommandLineArgs |> Array.skip 1 |> Array.toList with
    | ["--output"; path] -> Path.GetFullPath path
    | _ -> invalidArg "arguments" "Expected --output EMPTY_DIRECTORY."
if Directory.Exists output && not (Directory.EnumerateFileSystemEntries output |> Seq.isEmpty) then
    invalidArg "output" "Output must be empty; reference evidence is not overwritten."

type Reference = { Id: string; Title: string; UpperHeight: float; PhotoFirst: int; PhotoLast: int }
let references = [
    { Id = "double-parallelogram"; Title = "Four-legged double-parallelogram reference"; UpperHeight = 60
      PhotoFirst = 5425; PhotoLast = 5442 }
]

let leg height = MotionExamples.doubleParallelogram height 40 |> unwrap

let legs = ["front-right",0.0,0.0,-100.0; "rear-left",320.0,0.0,100.0
            "front-left",0.0,Math.PI,100.0; "rear-right",320.0,Math.PI,-100.0]

let robot height =
    let single = leg height
    let graph = Motion.compile single |> Result.defaultWith (String.concat "; " >> invalidOp)
    let bodies, hinges, sites, feet = ResizeArray<_>(),ResizeArray<_>(),ResizeArray<_>(),ResizeArray<_>()
    let name prefix id = if id = "chassis" then id else prefix+"-"+id
    for prefix,x,phase,_ in legs do
        let pose = (Motion.solve graph phase None |> unwrap).Poses
        let endpoint (p: MotionPort) =
            { Body = name prefix p.Body; Site = if p.Body = "chassis" then prefix+"-"+p.Site else p.Site }
        for body in single.Bodies do
            if body.Grounded then
                for site in body.Sites do sites.Add { site with Id = prefix+"-"+site.Id; Position = Vec3.add site.Position (Vec3.create x 0 0) }
            else
                bodies.Add { body with Id = name prefix body.Id
                                       Initial = { pose[body.Id] with Origin = Vec3.add pose[body.Id].Origin (Vec3.create x 0 0) } }
        for h in single.Hinges do hinges.Add { Id = prefix+"-"+h.Id; A = endpoint h.A; B = endpoint h.B }
        feet.Add (Motion.foot prefix (name prefix "shoe") "contact" 4)
    bodies.Add (Motion.ground "chassis" (List.ofSeq sites))
    { Bodies = List.ofSeq bodies; Hinges = List.ofSeq hinges; Feet = List.ofSeq feet
      Motor = { Body = "front-right-crank"; Phase = 0 }
      Transmissions = [for prefix,_,phase,_ in legs do
                          if prefix <> "front-right" then yield Motion.transmit "front-right-crank" (prefix+"-crank") 1 phase] }

let samples program =
    let graph = Motion.compile program |> Result.defaultWith (String.concat "; " >> invalidOp)
    graph, (Motion.sample graph [for i in 0..144 -> float i*Math.PI/36.0] |> unwrap)

let verify (program: MotionProgram) graph (frames: MotionFrame list) =
    let maxAngle body = frames |> List.map (fun frame -> abs frame.Poses[body].Angle) |> List.max
    let horizontal = legs |> List.map (fun (prefix,_,_,_) -> maxAngle (prefix+"-shoe")) |> List.max
    let upright = legs |> List.map (fun (prefix,_,_,_) -> maxAngle (prefix+"-upright-l")) |> List.max
    let maxResidual = frames |> List.map _.MaxResidual |> List.max
    let localPath prefix =
        let _,x,_,_ = legs |> List.find (fun (id,_,_,_) -> id = prefix)
        frames |> List.map (fun frame -> Vec3.subtract ((Motion.footPositions graph frame)[prefix]) (Vec3.create x 0 0))
    let diagonalError a b = List.map2 Vec3.distance (localPath a) (localPath b) |> List.max
    let oppositeError =
        let a,b = localPath "front-right",localPath "front-left"
        [for i in 0..108 -> Vec3.distance a[i+36] b[i]] |> List.max
    let repeatError =
        [for id,_,_,_ in legs ->
            let path = localPath id
            Vec3.distance path[0] path[72]] |> List.max
    let diagonal = max (diagonalError "front-right" "rear-left") (diagonalError "front-left" "rear-right")
    if [horizontal;upright;maxResidual;diagonal;oppositeError;repeatError] |> List.exists (fun e -> not (Double.IsFinite e) || e > 1e-6) then
        invalidOp $"Reference invariants failed: horizontal={horizontal}; upright={upright}; residual={maxResidual}; diagonal={diagonal}; half-turn={oppositeError}; repeat={repeatError}"
    {| sampledCrankTurns = 2; frames = frames.Length; maxConstraintResidualLdu = maxResidual
       maxShoeAngleRadians = horizontal; maxUprightAngleRadians = upright
       maxDiagonalPairErrorLdu = diagonal; maxHalfTurnPairErrorLdu = oppositeError; maxCycleRepeatErrorLdu = repeatError
       feet = [for foot in program.Feet do
                  let path = frames |> List.map (fun f -> (Motion.footPositions graph f)[foot.Id])
                  yield {| id = foot.Id; strideLdu = (path |> List.maxBy _.X).X-(path |> List.minBy _.X).X
                           liftLdu = (path |> List.maxBy _.Y).Y-(path |> List.minBy _.Y).Y |}] |}

let layout reference (program: MotionProgram) (frame: MotionFrame) =
    let parts = ResizeArray<WalkerPlacement>()
    let add id part color position rotation = parts.Add { Id=id; Part=part; Color=color; Position=position; Rotation=rotation }
    let point prefix site =
        let body = program.Bodies |> List.find (fun b -> b.Id = prefix)
        Motion.transform frame.Poses[prefix] (body.Sites |> List.find (fun s -> s.Id = site)).Position
    let atDepth z (p: Vec3) = { p with Z=z }
    let beam id part color z start finish =
        let center = Vec3.scale 0.5 (Vec3.add start finish) |> atDepth z
        add id part color center (Rotation.beam (Vec3.subtract finish start))
    for prefix,x,_,face in legs do
        let a body site = point (prefix+"-"+body) site
        beam (prefix+"-lower-arm") "32316.dat" 4 (face-20.0) (a "lower-arm" "root") (a "lower-arm" "tip")
        beam (prefix+"-upper-arm") "32316.dat" 14 (face+20.0) (a "upper-arm" "root") (a "upper-arm" "tip")
        let midpoint = a "upright-l" "corner"
        let top = a "upright-l" "top"
        // A composite L replaces the photo's molded L; extra holes are deliberately unused.
        let lDirection = Vec3.normalize (Vec3.subtract top midpoint)
        let lCenter = Vec3.add midpoint (Vec3.scale 40 lDirection) |> atDepth (face+40.0)
        add (prefix+"-upright-l-vertical") "32316.dat" 14 lCenter (Rotation.beam lDirection)
        beam (prefix+"-upright-l-horizontal") "32523.dat" 14 (face+20.0) midpoint (a "upright-l" "toe-link")
        beam (prefix+"-driven-leg") "32525.dat" 72 face (a "driven-leg" "top") (a "driven-leg" "bottom")
        beam (prefix+"-parallel-leg") "32525.dat" 14 face (a "parallel-leg" "top") (a "parallel-leg" "bottom")
        let crankRoot,crankTip = a "crank" "root",a "crank" "tip"
        add (prefix+"-crank") "60483.dat" 0 (atDepth (face-20.0) crankRoot) (Rotation.beam (Vec3.subtract crankTip crankRoot))
        let shoeCenter = Vec3.scale 0.5 (Vec3.add (a "shoe" "left") (a "shoe" "right"))
        add (prefix+"-shoe") "3701.dat" 14 (atDepth face (Vec3.add shoeCenter (Vec3.create 0 -10 0))) Rotation.identity
        let groundTop = Vec3.create (x-80.0) -reference.UpperHeight 0
        add (prefix+"-fixed-upright") "32316.dat" 71 (atDepth (face-40.0) (Vec3.add groundTop (Vec3.create 0 40 0))) (Rotation.beam (Vec3.create 0 1 0))
    for face in [-60.0;60.0] do
        // Technic bricks run along local X; their hole centers sit at local Y=10.
        add $"chassis-rail-{face}" "3703.dat" 14 (Vec3.create 140 10 face) Rotation.identity
    add "payload-battery" "58119.dat" 71 (Vec3.create 120 -55 0) Rotation.identity
    add "payload-receiver" "58123.dat" 71 (Vec3.create 290 -120 0) Rotation.identity
    add "motor-envelope" "58121.dat" 71 (Vec3.create -20 -140 0) Rotation.identity
    for index in 0..3 do
        add $"gear-envelope-{index}" "3648b.dat" 71 (Vec3.create -20 (-100.0+60.0*float index) -70) Rotation.identity
    add "motor-pinion-envelope" "3647.dat" 71 (Vec3.create -20 -140 -70) Rotation.identity
    List.ofSeq parts

let ldraw parts =
    let rows = [
        yield "0 Photo-informed walker layout study"
        yield "0 // NOT A BUILDABLE OR SIMULATION-READY MODEL. See reference.json and project docs/photo-references.md."
        yield "0 // Motor, battery, receiver and gears are layout proxies; shafts, pins and bracing are incomplete."
        yield "0 // Confirmed linkage topology; inferred dimensions; +Y down; LDU."
        for p: WalkerPlacement in parts do
            yield $"0 // instance: {p.Id}"
            let r = p.Rotation
            let values = [p.Position.X;p.Position.Y;p.Position.Z;r.X.X;r.Y.X;r.Z.X;r.X.Y;r.Y.Y;r.Z.Y;r.X.Z;r.Y.Z;r.Z.Z]
                         |> List.map number |> String.concat " "
            yield $"1 {p.Color} {values} {p.Part}"
    ]
    String.concat "\n" rows + "\n"

Directory.CreateDirectory output |> ignore
let summaries = [
    for reference in references do
        let program = robot reference.UpperHeight
        let graph,frames = samples program
        let evidence = verify program graph frames
        let directory = Path.Combine(output,reference.Id)
        MotionExport.writeTurns directory program 72 2 |> ignore
        MotionExport.writeTurns (Path.Combine(directory,"single-leg")) (leg reference.UpperHeight) 72 2 |> ignore
        let parts = layout reference program frames.Head
        for part in parts do
            Catalog.get part.Part |> ignore
            if not (Rotation.valid part.Rotation && Vec3.finite part.Position) then invalidOp $"Invalid placement {part.Id}"
            if part.Id.StartsWith("chassis-rail-") then
                let ports = (Catalog.get part.Part).Ports |> List.map (fun port ->
                    Vec3.add part.Position (Rotation.apply part.Rotation port.Position))
                if ports.Length <> 15 || ports |> List.exists (fun p -> abs (p.Y-20.0) > 1e-6 || abs (p.Z-part.Position.Z) > 1e-6)
                   || abs ((ports |> List.minBy _.X).X) > 1e-6 || abs ((ports |> List.maxBy _.X).X-280.0) > 1e-6 then
                    invalidOp $"Chassis rail is not horizontal at its intended holes: {part.Id}"
        File.WriteAllText(Path.Combine(directory,"layout-study.ldr"),ldraw parts)
        let metadata = {|
            schemaVersion = 1; title = reference.Title; classification = "photo-informed-kinematic-reference-not-native-candidate"
            photoFiles = [for n in reference.PhotoFirst..reference.PhotoLast -> $"WalkerPhotos/IMG_{n}.HEIC"]
            confirmedByBuilder = [
                "One continuously rotating motor; gait timing is mechanical."
                "Front-right = rear-left; front-left = rear-right; diagonal pairs half a cycle apart."
                "Shared revolute corner joins lower arm, driven long leg and upright L."
                "The L's short horizontal branch points away from the chassis/red arm; the second long leg is on that outer side."
                "Crank drives an intermediate point of the long leg."
                "Upper parallelogram preserves the L orientation; lower parallelogram preserves the shoe orientation."
                "Both photo builds are one robot design for this project; phase-adjustment hardware is out of scope."
                "No worm gearing in the yellow/red robot."
            ]
            estimatesLdu = {| crankRadius=20; upperArmLength=80; drivenLegLength=200; crankAttachmentFromTop=80
                              shoePivotSpacing=40; uprightHeight=reference.UpperHeight; frontRearSpacing=320; trackWidth=200
                              fixedLowerToCrank=[60;80] |}
            assumptions = [
                "Metric dimensions, lateral part layering and initial configuration are approximations, not recovered measurements."
                "Upper L is rendered as separate straight beams, not an identified molded part or certified rigid assembly."
                "Catalogue 58121 is a PF XL motor, consistent with the visible housing; mounting, speed, torque and mass calibration are unconfirmed."
                "The kinematic input is the output crank angle; upstream motor reduction is unresolved, so sampled turns are crank turns, not calibrated physical motor turns."
                "Visible gear positions suggest a vertical reduction stack; tooth counts, compound stages and horizontal distribution are not reconstructed."
                "Gear and payload placements are visual envelopes only; numerical total mass/COM is deliberately not claimed."
                "One shared layout approximation covers both photo builds; aesthetics and removable battery hardware are not reconstructed."
                "Broad photo shoes are represented by a small brick proxy; native bounded-contact policy has not been reconciled."
                "No connectors, structural bracing, gearing realization or collision acceptance is certified."
            ]
            kinematicVerification = evidence; parts = parts
            nativeCandidate = false; reconstructionChecks = 0; dynamicTrials = 0
        |}
        File.WriteAllText(Path.Combine(directory,"reference.json"),Realization.json metadata+"\n")
        printfn "%s: max residual=%g LDU; horizontal shoe error=%g rad; feet=%d"
            reference.Id evidence.maxConstraintResidualLdu evidence.maxShoeAngleRadians evidence.feet.Length
        yield {| id=reference.Id; title=reference.Title; verification=evidence; dynamicTrials=0 |}
]
File.WriteAllText(Path.Combine(output,"references.json"),Realization.json {|schemaVersion=1;references=summaries;dynamicTrials=0|}+"\n")
let links =
    summaries |> List.map (fun r ->
        $"""<section><h2>{r.title}</h2><p><a href="{r.id}/preview.html">Interactive planar mechanism</a> |
<a href="{r.id}/layout-study.ldr">Approximate LDraw layout</a> |
<a href="{r.id}/single-leg/preview.html">Single-leg mechanism</a> |
<a href="{r.id}/reference.json">Evidence and assumptions</a></p>
<img src="{r.id}/cycle.svg" alt="Four motor phases and foot trajectories" style="width:100%%"></section>""") |> String.concat "\n"
File.WriteAllText(Path.Combine(output,"index.html"), $"""<!doctype html><html lang="en"><meta charset="utf-8">
<title>Photo-informed four-legged walker references</title>
<style>body{{font:16px system-ui;max-width:1250px;margin:2rem auto;padding:1rem;background:#eef2f7;color:#203147}}section{{background:white;padding:1rem;margin:1rem 0;border-radius:12px}}a{{color:#1768a6}}</style>
<h1>Photo-informed walker references</h1>
<p><strong>Kinematic approximations, not reconstructed LEGO builds or simulated walkers.</strong>
Confirmed double-parallelogram topology and diagonal-pair timing; estimated dimensions.
All four shoes remain horizontal over two sampled crank revolutions. The input shown as motor angle in the generic preview is the output crank angle here; upstream reduction is unresolved.
No dynamics, reconstruction or mass/COM certification.</p>
<p>The left/right legs overlap in the planar diagram; the LDraw layout separates them in depth.
Drivetrain graphics are layout proxies, not physical gear-train verification.</p>{links}</html>""")
