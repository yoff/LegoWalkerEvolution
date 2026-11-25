#r "../src/LegoWalkers.Core/bin/Debug/net10.0/LegoWalkers.Core.dll"

open System
open System.IO
open System.Text.Json
open LegoWalkers

type Triangle = { Part: string; A: float; B: float; C: float }
type Dimensions = {
    Upper: Triangle; Foot: Triangle; GroundX: float; GroundY: float
    Crank: float; UpperDrive: float; LowerDrive: float; Rocker: float; Shin: float
}
type Geometry = { O: Vec3; Y: Vec3; Z: Vec3; X: Vec3; W: Vec3; V: Vec3; U: Vec3; T: Vec3 }
type Metrics = {
    Stride: float; Lift: float; StanceFraction: float; StanceTravel: float
    MaximumStep: float; Score: float
}
type Candidate = { Dimensions: Dimensions; Metrics: Metrics; Initial: Geometry }

let output =
    match fsi.CommandLineArgs |> Array.skip 1 |> Array.toList with
    | ["--output"; path] -> Path.GetFullPath path
    | _ -> invalidArg "arguments" "Expected --output EMPTY_DIRECTORY."
if Directory.Exists output && not (Directory.EnumerateFileSystemEntries output |> Seq.isEmpty) then
    invalidArg "output" "Search output must be empty."

let point x y = Vec3.create x y 0
let triangles foot =
    [for part in ["32526.dat"; "32271.dat"] do
        let ports = (Catalog.get part).Ports
        for a in ports do
          for b in ports do
            for c in ports do
                let ab,ac = Vec3.subtract b.Position a.Position,Vec3.subtract c.Position a.Position
                if a.Kind=RoundHole && b.Kind=RoundHole && (foot || c.Kind=RoundHole)
                   && Vec3.length (Vec3.cross ab ac)>1e-6 then
                    let triangle = {
                        Part=part; A=Vec3.length ab; B=Vec3.length ac
                        C=Vec3.distance b.Position c.Position
                    }
                    let cosine = Vec3.dot ab ac/(triangle.A*triangle.B)
                    if (if foot then cosine>= -0.65 && cosine<=0.25
                        else abs cosine<=0.5 && abs(triangle.A/triangle.B-41.5/40.1)<0.6)
                       && triangle.A>=20.0 && triangle.B>=20.0 then
                        yield triangle]
    |> List.distinct

let meet (p: Vec3) (r: float) (q: Vec3) (s: float) =
    let delta = Vec3.subtract q p
    let distance = Vec3.length delta
    if distance<1e-8 || distance>=r+s || distance<=abs(r-s) then None
    else
        let along = (r*r-s*s+distance*distance)/(2.0*distance)
        let square = r*r-along*along
        if square<0.01 then None
        else
            let axis = Vec3.scale (1.0/distance) delta
            let center = Vec3.add p (Vec3.scale along axis)
            let offset = Vec3.scale (sqrt square) (point -axis.Y axis.X)
            Some [Vec3.add center offset; Vec3.subtract center offset]

let geometry (d: Dimensions) angle (previous: Geometry option) =
    let o,y,z = point 0 0,point -d.GroundX -d.GroundY,point (d.Crank*cos angle) (d.Crank*sin angle)
    let choose prior initial values =
        match previous with
        | Some frame -> values |> List.minBy (fun value -> Vec3.distance (prior frame) value)
        | None -> initial values
    match meet z d.UpperDrive y d.Upper.A,meet z d.LowerDrive y d.Rocker with
    | Some xs,Some ws ->
        let x = choose _.X (List.maxBy _.Y) xs
        let w = choose _.W (List.minBy _.Y) ws
        match meet y d.Upper.B x d.Upper.C with
        | None -> None
        | Some vs ->
            let v = choose _.V (List.minBy _.X) vs
            match meet v d.Shin w d.Foot.A with
            | None -> None
            | Some us ->
                let u = choose _.U (List.minBy _.X) us
                match meet w d.Foot.B u d.Foot.C with
                | None -> None
                | Some ts ->
                    let t = choose _.T (List.minBy _.Y) ts
                    Some {O=o;Y=y;Z=z;X=x;W=w;V=v;U=u;T=t}
    | _ -> None

let points (g: Geometry) = [g.O;g.Y;g.Z;g.X;g.W;g.V;g.U;g.T]
let downward (p: Vec3) = point p.X -p.Y
let program (g: Geometry) =
    let sites =
        Map.ofList [
            "ground",Map.ofList ["drive",g.O;"rocker",g.Y]
            "crank",Map.ofList ["root",g.O;"tip",g.Z]
            "upper-drive",Map.ofList ["crank",g.Z;"rocker",g.X]
            "lower-drive",Map.ofList ["crank",g.Z;"rocker",g.W]
            "upper-triangle",Map.ofList ["pivot",g.Y;"drive",g.X;"knee",g.V]
            "rocker",Map.ofList ["root",g.Y;"tip",g.W]
            "shin",Map.ofList ["upper",g.V;"lower",g.U]
            "foot-triangle",Map.ofList ["pivot",g.W;"knee",g.U;"toe",g.T]
        ]
    { MotionExamples.jansen with
        Bodies = MotionExamples.jansen.Bodies |> List.map (fun body ->
            let origin = if body.Grounded then Vec3.zero else downward (sites[body.Id][body.Sites.Head.Id])
            {body with Initial={Origin=origin;Angle=0}
                       Sites=body.Sites |> List.map (fun site ->
                           {site with Position=Vec3.subtract (downward (sites[body.Id][site.Id])) origin})}) }

let neighboring target =
    let center = int (Math.Round(target/20.0))
    [for n in center-1..center+1 do if n>=1 && n<=10 then yield float n*20.0]

let measure (feet: Vec3 array) maximumStep =
    let span (values: Vec3 array) = (values |> Array.maxBy _.X).X-(values |> Array.minBy _.X).X
    let low = (feet |> Array.maxBy _.Y).Y
    let stance = feet |> Array.filter (fun foot -> foot.Y>=low-2.0)
    let stride,lift,fraction,travel =
        span feet,low-(feet |> Array.minBy _.Y).Y,float stance.Length/float feet.Length,span stance
    if stride<40.0 || lift<10.0 || lift>60.0 || fraction<0.25 || travel<40.0 || maximumStep>20.0 then None
    else Some {Stride=stride;Lift=lift;StanceFraction=fraction;StanceTravel=travel
               MaximumStep=maximumStep;Score=travel*fraction-maximumStep*0.1}

let evaluate dimensions =
    let frames = ResizeArray<Geometry>()
    let mutable previous = None
    let mutable valid = true
    let mutable n = 0
    while valid && n<=72 do
        match geometry dimensions (-float n*2.0*Math.PI/72.0) previous with
        | None -> valid <- false
        | Some frame ->
            frames.Add frame
            previous <- Some frame
        n <- n+1
    if not valid then None
    elif List.zip (points (frames[0])) (points (frames[72])) |> List.exists (fun (a,b) -> Vec3.distance a b>1e-6) then None
    else
        let feet = frames |> Seq.take 72 |> Seq.map (_.T >> downward) |> Seq.toArray
        let maximumStep =
            frames |> Seq.pairwise |> Seq.collect (fun (a,b) ->
                List.zip (points a) (points b) |> List.map (fun (a,b) -> Vec3.distance a b)) |> Seq.max
        measure feet maximumStep |> Option.map (fun metrics ->
            {Dimensions=dimensions;Initial=frames[0];Metrics=metrics})

let upper = triangles false
let feet = triangles true
printfn "Searching %d upper and %d foot stock socket triples." upper.Length feet.Length
let mutable attempted = 0
let eligible = ResizeArray<Candidate>()
for a in upper do
    let scale = (a.A+a.B)/(41.5+40.1)
    if scale>=0.9 && scale<=1.7 then
        let suitableFoot = feet |> List.filter (fun foot ->
            List.zip [foot.A;foot.B;foot.C] [36.7*scale;49.0*scale;65.7*scale]
            |> List.forall (fun (actual,target) -> abs(actual/target-1.0)<=0.35))
        for foot in suitableFoot do
          for gx in neighboring (38.0*scale) do
            for gy in [0.0;20.0] do
              for ud in neighboring (50.0*scale) do
                for ld in neighboring (61.9*scale) do
                  for rocker in neighboring (39.3*scale) do
                    for shin in neighboring (39.4*scale) do
                        let d = {Upper=a;Foot=foot;GroundX=gx;GroundY=gy;Crank=20.0
                                 UpperDrive=ud;LowerDrive=ld;Rocker=rocker;Shin=shin}
                        attempted <- attempted+1
                        match evaluate d with
                        | Some candidate -> eligible.Add candidate
                        | None -> ()
printfn "%d dimensional combinations; %d pass the coarse cycle and stance filter." attempted eligible.Count
if eligible.Count=0 then invalidOp "No stock-length candidate met the declared kinematic thresholds."

let fits (candidate: Candidate) (p: MotionProgram) =
    p.Bodies |> List.map (fun body ->
        let positions = body.Sites |> List.map (fun site -> Motion.transform body.Initial site.Position)
        let choices =
            match body.Id with
            | "upper-triangle" -> [candidate.Dimensions.Upper.Part]
            | "foot-triangle" -> [candidate.Dimensions.Foot.Part]
            | "crank" -> ["60483.dat";"6632.dat"]
            | "ground" -> Catalog.planarBeamParts @ ["32526.dat"]
            | _ -> Catalog.planarBeamParts
        let matches = [
            for part in choices do
              for position,rotation in Catalog.fitPlanarPart part (Vec3.create 0 0 1) positions do
                let sockets = (Catalog.get part).Ports |> List.map (fun socket ->
                    socket.Kind,Vec3.add position (Rotation.apply rotation socket.Position))
                if body.Id<>"crank" || sockets |> List.exists (fun (kind,point) ->
                    kind=AxleHole && Vec3.distance point body.Initial.Origin<1e-6) then
                    yield body.Id,part,position,rotation
        ]
        matches |> List.tryHead)
    |> fun choices -> if choices |> List.exists Option.isNone then None else Some (choices |> List.map Option.get)

let verify candidate =
    let p = program candidate.Initial
    match fits candidate p,Motion.compile p with
    | Some parts,Ok graph ->
        let solve direction =
            Motion.sample graph [for n in 0..1440 -> direction*float n*2.0*Math.PI/720.0]
            |> Result.bind (fun frames ->
                let first,last = frames.Head,List.last frames
                let closed = p.Bodies |> List.forall (fun body ->
                    let a,b = first.Poses[body.Id],last.Poses[body.Id]
                    Vec3.distance a.Origin b.Origin<1e-5
                    && abs(atan2 (sin(a.Angle-b.Angle)) (cos(a.Angle-b.Angle)))<1e-6)
                if closed then Ok frames else Error "Generic solver changed assembly branch over two turns.")
        match solve 1.0,solve -1.0 with
        | Ok forward,Ok backward ->
            let residual = forward @ backward |> List.maxBy _.MaxResidual |> _.MaxResidual
            let world (frame: MotionFrame) =
                p.Bodies |> List.collect (fun body -> body.Sites |> List.map (fun site ->
                    Motion.transform frame.Poses[body.Id] site.Position))
            let maximumStep =
                forward |> List.indexed |> List.choose (fun (index,frame) -> if index%10=0 then Some frame else None)
                |> List.pairwise |> List.collect (fun (a,b) ->
                    List.zip (world a) (world b) |> List.map (fun (a,b) -> Vec3.distance a b)) |> List.max
            let feet = forward |> List.take 720 |> List.map (fun frame -> (Motion.footPositions graph frame)["toe"]) |> List.toArray
            match measure feet maximumStep with
            | Some metrics -> Some ({candidate with Metrics=metrics},p,parts,residual)
            | None ->
                printfn "Fine-sampled generic motion missed the declared stance thresholds."
                None
        | Error message,_ | _,Error message ->
            printfn "Generic motion rejection: %s" message
            None
    | None,_ ->
        printfn "Stock socket-fit rejection."
        None
    | _,Error errors -> invalidOp (String.concat "; " errors)

let ranked = eligible |> Seq.sortByDescending _.Metrics.Score |> Seq.truncate 30 |> Seq.toList
let finalists = ranked |> List.choose verify |> List.sortByDescending (fun (candidate,_,_,_) -> candidate.Metrics.Score)
if finalists.IsEmpty then invalidOp "No finalist passed exact catalog fitting and independent generic motion verification."
let winner,p,parts,residual = finalists.Head
Directory.CreateDirectory(output) |> ignore
let preview,_ = MotionExport.writeTurns (Path.Combine(output,"motion")) p 720 2
let options = JsonSerializerOptions(PropertyNamingPolicy=JsonNamingPolicy.CamelCase,WriteIndented=true)
let write name value = File.WriteAllText(Path.Combine(output,name),JsonSerializer.Serialize(value,options)+"\n")
let previewFile = Path.Combine(preview,"preview.html")
let heading =
    "<h1>Stock-compatible Jansen leg: motion study</h1>"
    + $"<p>{attempted} dimensional combinations; {finalists.Length} independently verified finalists. "
    + "Rigid three-point bodies fit molded stock beams. Pin routing, depth layers and collision admission are not yet established. "
    + "<a href=\"../search.json\">Search evidence and stock placements</a> &middot; "
    + "<a href=\"../stock-fit.ldr\">Exploded stock-fit LDraw study</a></p>"
File.WriteAllText(previewFile,File.ReadAllText(previewFile).Replace("<h1>Motion-construction grammar</h1>",heading))
write "search.json" {|
    schemaVersion=1; mode="stock-socket-search-not-assembled-or-powered"
    seedProgramSha256=Export.sha256 (MotionExport.programJson MotionExamples.jansen)
    compilerSha256=System.Security.Cryptography.SHA256.HashData(File.ReadAllBytes(typeof<MotionProgram>.Assembly.Location))
                   |> Convert.ToHexString |> _.ToLowerInvariant()
    attempted=attempted; coarseAccepted=eligible.Count; verifiedFinalists=finalists.Length
    thresholds={| strideLdu=40;liftMinLdu=10;liftMaxLdu=60;stanceBandLdu=2;stanceFraction=0.25;stanceTravelLdu=40;maximumFiveDegreeStepLdu=20 |}
    chosen=winner; maxResidualLdu=residual
    parts=parts |> List.map (fun (body,part,position,rotation) -> {|body=body;part=part;position=position;rotation=rotation|})
    finalists=finalists |> List.map (fun (candidate,_,_,residual) -> {|candidate=candidate;maxResidualLdu=residual|})
|}
let stock = parts |> List.mapi (fun index (body,part,position,rotation) ->
    {Id=body+"-part";Part=part;Position={position with Z=float index*20.0};Rotation=rotation
     Color=[|71;14;4;2;22;25;11;19|][index];Body=Some body})
let assembly = {
    Kind=Leg;Genome=Genome.defaults;Parts=stock;Joints=[];Gears=[];Drives=[];Feet=[];FootPaths=[]
    Chassis="ground";Warnings=["Stock socket-fit study only; no assembled pivots or collision admission."]
}
File.WriteAllText(Path.Combine(output,"stock-fit.ldr"),
    "0 Stock socket-fit study: exploded depth layers; no pins, motor, axial retention or collision admission.\n"+Export.ldraw assembly)
printfn "Winner: %A" winner.Dimensions
printfn "Stride %.2f LDU; lift %.2f LDU; low-band travel %.2f LDU over %.1f%% of cycle; residual %.3g LDU."
    winner.Metrics.Stride winner.Metrics.Lift winner.Metrics.StanceTravel (100.0*winner.Metrics.StanceFraction) residual
printfn "Preview: %s" preview
