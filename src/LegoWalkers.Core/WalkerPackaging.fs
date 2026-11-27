namespace LegoWalkers

open System
open System.Collections.Generic

type WalkerComponentTarget = {
    Placement: WalkerPlacement
    ToleranceLdu: Vec3
}

type WalkerDriveRealization = {
    Root: string
    Gears: (string * WalkerPlacement) list
    Axles: (string * WalkerPlacement) list
    MotorClockScale: float
    InternalCompoundBearings: bool
}

type WalkerComponentMass = {
    Part: string
    MassKg: float
    CenterLdu: Vec3
}

type WalkerBalanceModel = {
    TotalMassKg: float
    CenterOfMassLdu: Vec3
    SupportCenterLdu: Vec3
    SupportHalfSpanLdu: Vec3
    Components: WalkerComponentMass list
}

type WalkerPackagingRequest = {
    Reference: MotionProgram
    Program: MotionProgram
    Source: WalkerGenome
    Targets: WalkerComponentTarget list
    Drive: WalkerDriveRealization
    ObsoletePartIds: string list
    GroundClearance: MotionGroundClearance option
    FootConstructionGates: GateDecision list
    Balance: WalkerBalanceModel option
}

type WalkerPackagingConstruction = {
    Screened: ScreenedMotion
    Realized: RealizedWalker
    AddedPartIds: string list
    RetainedFramePartIds: string list
    RemovedPartIds: string list
    Derivation: string list
    Clearance: WalkerClearanceReport
}

type WalkerPackagingAttempt = {
    Orientation: int
    Construction: WalkerPackagingConstruction option
    Gates: GateDecision list
}

type WalkerConstructionInspection = {
    Goal: string
    Rule: string
    Fixed: WalkerMotionSpace.Envelope list
    Candidate: WalkerMotionSpace.Envelope list
    Reserved: WalkerMotionSpace.Envelope list
    Conflicts: WalkerMotionSpace.Conflict list
    Checked: int
    Rejected: int
    CacheHits: int
}

module WalkerPackaging =
    let version = "packaging-realization-v21"
    let maximumAttempts = 2
    let maximumAnchorChoices = 64
    let private zAxis = Vec3.create 0 0 1
    let private stock = ["32523.dat",40.0;"32316.dat",80.0;"32524.dat",120.0;"32525.dat",200.0]
    let private bearingGap (gear: WalkerPlacement) =
        if Catalog.gearTeeth gear.Part=Some 40 then 30.0 else 20.0
    let minimumBearingSpan first last = bearingGap first+bearingGap last+20.0
    let bearingPlanes internalPair (gears: WalkerPlacement list) =
        match gears |> List.sortBy _.Position.Z with
        | [first;last] when internalPair ->
            if last.Position.Z-first.Position.Z<minimumBearingSpan first last-1e-6 then
                invalidArg "gears" "Internal compound bearings require gear clearance and at least 20 LDU between bearing centers."
            [first.Position.Z+bearingGap first;last.Position.Z-bearingGap last]
        | gears -> gears |> List.collect (fun gear -> [gear.Position.Z-bearingGap gear;gear.Position.Z+bearingGap gear])
    let private error code diagnostics =
        { Gate="packaging-realization"; Outcome="reject"; Code=code; Diagnostics=diagnostics }

    let private groundConflicts request screened genome =
        request.GroundClearance |> Option.map (fun requirements ->
            WalkerMotionSpace.groundPlan requirements screened genome
            |> fun plan -> WalkerMotionSpace.auditGround plan screened genome) |> Option.defaultValue []

    let driveBearingPlanes (request: WalkerPackagingRequest) body gears =
        let retainers =
            request.Source.Bodies |> List.tryFind (fun b -> b.Body=body)
            |> Option.map _.Parts |> Option.defaultValue []
            |> List.filter (fun p -> List.contains p.Part ["3713.dat";"4265c.dat"]
                                    && not(List.contains p.Id request.ObsoletePartIds)
                                    && (gears |> List.exists (fun (gear: WalkerPlacement) ->
                                        abs(p.Position.X-gear.Position.X)<1e-6 && abs(p.Position.Y-gear.Position.Y)<1e-6)))
        bearingPlanes request.Drive.InternalCompoundBearings gears |> List.map (fun plane ->
            let gear = gears |> List.minBy (fun gear -> abs(plane-gear.Position.Z))
            let direction = if plane>gear.Position.Z then 1.0 else -1.0
            [0..2*retainers.Length] |> List.map (fun step -> plane+direction*20.0*float step)
            |> List.find (fun candidate ->
                retainers |> List.forall (fun collar ->
                    abs(candidate-collar.Position.Z)>=10.0+Catalog.socketHalfThickness collar.Part-1e-6)))

    let private ports (part: WalkerPlacement) =
        (Catalog.get part.Part).Ports |> List.map (fun port ->
            port,Vec3.add part.Position (Rotation.apply part.Rotation port.Position),
            Rotation.apply part.Rotation port.Axis)

    let private mountPorts (part: WalkerPlacement) =
        Catalog.mountPorts part.Part |> List.map (fun port ->
            port,Vec3.add part.Position (Rotation.apply part.Rotation port.Position),
            Rotation.apply part.Rotation port.Axis)

    let private beam id part length (a: Vec3) (b: Vec3) plane =
        let a,b = {a with Z=plane},{b with Z=plane}
        let rotation = Rotation.beam (Vec3.subtract b a)
        {Id=id;Part=part;Color=7;Position=Vec3.scale 0.5 (Vec3.add a b);Rotation=rotation}
        |> fun p ->
            if abs(Vec3.distance a b-length)>1e-6 then invalidArg "support" "Beam span differs from its socket span."
            p

    let private pinAlong id axis (a: Vec3) (b: Vec3) =
        if Vec3.length(Vec3.cross (Vec3.subtract b a) axis)>1e-6 then
            invalidArg "support" "Mount sockets must be coaxial."
        // Each supported socket wall occupies at most +/-10 LDU around its center.
        let reach = Vec3.distance a b+20.0
        match ["4459.dat";"6558.dat"] |> List.tryFind (fun stock ->
            let low,high = Catalog.shaftRange stock
            high-low>=reach-1e-6) with
        | None -> Error(error "unsupported-mount-span" [$"Socket wall span {reach:G6} LDU exceeds supported friction pins."])
        | Some stock ->
            let y = if abs axis.Z>0.999 then Vec3.create 1 0 0 else Vec3.create 0 1 0
            let rotation = {X=axis;Y=y;Z=Vec3.cross axis y}
            Ok {Id=id;Part=stock;Color=8;Position=Vec3.scale 0.5 (Vec3.add a b);Rotation=rotation}

    let private pin id a b = pinAlong id zAxis a b

    let motorMountOptions (motor: WalkerPlacement) =
        if motor.Part<>"58121.dat" then invalidArg "motor" "Motor mounting motifs require the supported PF motor."
        [for name,depth,supportDepth in ["front-forward",10.0,-10.0;"front-rearward",10.0,30.0;"rear-saddle",110.0,130.0] do
            let sockets = (Catalog.get motor.Part).Ports |> List.filter (fun p ->
                p.Kind=RoundHole && abs p.Axis.X>0.999 && abs(p.Position.Z-depth)<1e-6)
            let parts = [
                for index,port in sockets |> List.indexed do
                    let prefix = $"packaging-{motor.Id}-{index}"
                    let socket = Vec3.add motor.Position (Rotation.apply motor.Rotation port.Position)
                    let axis = Rotation.apply motor.Rotation port.Axis
                    let rotation = Rotation.aroundZ (if axis.X>0.0 then 0.0 else Math.PI)
                    let origin = Vec3.add socket (Vec3.scale 20.0 axis)
                    let corner = {Id=prefix+"-corner";Part="6536.dat";Color=7;Position=origin;Rotation=rotation}
                    let mount = Vec3.add origin (Rotation.apply rotation (Vec3.create 0 20 0))
                    let plane = motor.Position.Z+supportDepth*motor.Rotation.Z.Z
                    let covered result = result |> Result.defaultWith (Realization.json >> invalidOp)
                    yield corner
                    yield pin (prefix+"-mount-pin") {mount with Z=plane} mount |> covered
                    yield pinAlong (prefix+"-side-pin") axis socket origin |> covered
            ]
            yield name,parts]

    let contactCapacityDeficit (pin: WalkerPlacement) (parts: WalkerPlacement list) extraContacts =
        let low,high = Catalog.shaftRange pin.Part
        let capacity = max 2 (int(Math.Round((high-low)/20.0)))
        let contacts = parts |> List.distinctBy _.Id |> List.filter (fun part ->
            ports part |> List.exists (fun (_,point,axis) ->
                let delta = Vec3.subtract point pin.Position
                let along = Vec3.dot delta pin.Rotation.X
                abs(Vec3.dot axis pin.Rotation.X)>1.0-1e-6
                && Vec3.length(Vec3.cross delta pin.Rotation.X)<=3.0+1e-6
                && along>=low-8.0-1e-6 && along<=high+8.0+1e-6)) |> List.length
        max 0 (contacts+extraContacts-capacity)

    let private twoBeamSupportRoutes orientation prefix (anchors: Vec3 list) (occupied: WalkerPlacement list) (socket: Vec3) (plane: float) =
        let routesToPlane =
            anchors |> List.collect (fun a ->
                [plane-20.0;plane+20.0;plane-40.0;plane+40.0] |> List.choose (fun firstPlane ->
                    if abs(a.Z-firstPlane)<=40.0+1e-6
                       && Vec3.distance {a with Z=0.0} {socket with Z=0.0}>20.0 then Some(a,firstPlane)
                    else None))
            |> List.sortBy (fun (a,firstPlane) ->
                Vec3.distance {a with Z=0.0} {socket with Z=0.0},abs(a.Z-firstPlane),a.X,a.Y,a.Z,firstPlane)
            |> List.truncate maximumAnchorChoices
        let routes = [
            for anchor,firstPlane in routesToPlane do
                let delta = Vec3.subtract {socket with Z=0.0} {anchor with Z=0.0}
                let distance = Vec3.length delta
                for partA,lengthA in stock do
                    for partB,lengthB in stock do
                        if distance<lengthA+lengthB-1e-6 && distance>abs(lengthA-lengthB)+1e-6 then
                            let along = (lengthA*lengthA-lengthB*lengthB+distance*distance)/(2.0*distance)
                            let height = sqrt(max 0.0 (lengthA*lengthA-along*along))
                            let direction = Vec3.scale (1.0/distance) delta
                            let perpendicular = Vec3.create -direction.Y direction.X 0
                            let elbow = Vec3.add {anchor with Z=0.0}
                                           (Vec3.add (Vec3.scale along direction) (Vec3.scale (float orientation*height) perpendicular))
                            let a = beam (prefix+"-anchor-beam") partA lengthA anchor elbow firstPlane
                            let b = beam (prefix+"-socket-beam") partB lengthB elbow socket plane
                            let pins = [
                                pin (prefix+"-anchor-pin") anchor {anchor with Z=firstPlane}
                                pin (prefix+"-elbow-pin") {elbow with Z=firstPlane} {elbow with Z=plane}
                            ]
                            if pins |> List.forall Result.isOk then
                                let pins = pins |> List.map (Result.defaultWith (fun _ -> invalidOp "Unreachable invalid support pin."))
                                let free = pins |> List.forall (fun p ->
                                    occupied |> List.forall (fun other ->
                                        not(Catalog.shaftsOverlap p.Part p.Position p.Rotation other.Part other.Position other.Rotation)))
                                if free then yield lengthA+lengthB,anchor,a,b,pins
        ]
        routes |> List.sortBy (fun (length,anchor,_,_,_) -> length,anchor.X,anchor.Y,anchor.Z)
        |> List.map (fun (_,_,a,b,pins) -> [a;b],pins)

    let private routesWithCount acceptsPartial orientation prefix anchors occupied count socket plane =
        let rec routesWith count socket plane =
            if count=2 then twoBeamSupportRoutes orientation prefix anchors occupied socket plane |> Seq.ofList
            else
                seq {
                    for offset in [Vec3.create 40 0 0;Vec3.create -40 0 0;Vec3.create 0 40 0;Vec3.create 0 -40 0] do
                        for bridgePlane in [plane-40.0;plane+40.0] do
                            let bridge = {Vec3.add socket offset with Z=bridgePlane}
                            let extension = beam (prefix + $"-bridge-beam-{count}") "32523.dat" 40.0 bridge socket plane
                            match pin (prefix + $"-bridge-pin-{count}") bridge {bridge with Z=plane} with
                            | Error _ -> ()
                            | Ok connector when acceptsPartial ([extension],[connector]) ->
                                for parts,pins in routesWith (count-1) bridge bridgePlane do
                                    if occupied @ pins |> List.forall (fun other ->
                                        not(Catalog.shaftsOverlap connector.Part connector.Position connector.Rotation
                                                                    other.Part other.Position other.Rotation)) then
                                        yield parts @ [extension],pins @ [connector]
                            | Ok _ -> ()
                }
        routesWith count socket plane

    let supportRoutes orientation prefix anchors occupied socket plane =
        [2..5] |> List.tryPick (fun count ->
            let routes = routesWithCount (fun _ -> true) orientation prefix anchors occupied count socket plane |> Seq.toList
            if routes.IsEmpty then None else Some routes) |> Option.defaultValue []

    let maximumMotionRoutesPerTopology = 512
    let private boundedMotionRoutes acceptsPartial accepts orientation prefix anchors occupied socket plane =
        seq {
            for count in 2..5 do
                yield! routesWithCount acceptsPartial orientation prefix anchors occupied count socket plane
                       |> Seq.truncate maximumMotionRoutesPerTopology |> Seq.filter accepts
        } |> Seq.truncate 8 |> Seq.toList

    let motionSupportRoutes accepts = boundedMotionRoutes (fun _ -> true) accepts

    /// The predicate must reject each unsafe part independently, including incomplete routes.
    let motionSupportRoutesWithPruning accepts = boundedMotionRoutes accepts accepts

    let private support orientation prefix anchors occupied socket plane =
        match supportRoutes orientation prefix anchors occupied socket plane |> List.tryHead with
        | None -> Error(error "no-bounded-support-route" [$"No covered stock-beam support route for {prefix} at {socket}."])
        | Some route -> Ok route

    let private validateDrive compound (request: WalkerPackagingRequest) =
        let sourceParts =
            (request.Source.Bodies |> List.collect _.Parts) @ request.Source.Mounts
            @ (request.Source.Connections |> List.map _.Placement)
        let known = sourceParts |> List.map _.Id |> Set.ofList
        let expected = Set.ofList ["payload-battery";"payload-receiver";request.Source.MotorPart]
        let targets = request.Targets |> List.map (fun t -> t.Placement.Id)
        let driveBodies = request.Program.Motor.Body::(request.Program.Transmissions |> List.collect (fun t -> [t.Input;t.Output])) |> Set.ofList
        let stockIds = (request.Drive.Gears @ request.Drive.Axles) |> List.map (snd >> _.Id)
        let invalidBalance = request.Balance |> Option.exists (fun model ->
            isNull(box model) || isNull(box model.Components)
            || not(Double.IsFinite model.TotalMassKg) || model.TotalMassKg<=0.0
            || not(Vec3.finite model.CenterOfMassLdu && Vec3.finite model.SupportCenterLdu && Vec3.finite model.SupportHalfSpanLdu)
            || model.SupportHalfSpanLdu.X<=0.0 || model.SupportHalfSpanLdu.Z<=0.0
            || model.Components |> List.exists (fun c -> isNull(box c))
            || (model.Components |> List.map _.Part |> Set.ofList)<>expected || model.Components.Length<>3
            || model.Components |> List.exists (fun c -> not(Double.IsFinite c.MassKg) || c.MassKg<=0.0 || not(Vec3.finite c.CenterLdu))
            || (model.Components |> List.sumBy _.MassKg)>model.TotalMassKg+1e-9)
        if invalidBalance then
            Error(error "invalid-balance-model" ["Balance estimates require positive mass, finite centers/support spans and exactly the three hardware component masses."])
        elif targets.Length<>3 || Set.ofList targets<>expected then
            Error(error "invalid-component-targets" ["Exactly one battery, receiver and motor target is required."])
        elif request.Targets |> List.exists (fun t ->
            not(Vec3.finite t.Placement.Position && Rotation.valid t.Placement.Rotation && Vec3.finite t.ToleranceLdu)
            || t.ToleranceLdu.X<0.0 || t.ToleranceLdu.Y<0.0 || t.ToleranceLdu.Z<0.0
            || (Catalog.get t.Placement.Part).Connector.IsSome) then
            Error(error "invalid-component-targets" ["Finite structural placements and nonnegative tolerances are required."])
        elif request.Targets |> List.exists (fun t ->
            let expected = if t.Placement.Id="payload-battery" then "58119.dat"
                           elif t.Placement.Id="payload-receiver" then "58123.dat" else "58121.dat"
            t.Placement.Part<>expected) then
            Error(error "invalid-component-targets" ["Hardware targets must use their required catalogue parts."])
        elif request.Targets |> List.exists (fun t ->
            mountPorts t.Placement |> List.exists (fun (_,_,axis) ->
                if t.Placement.Part="58121.dat" then abs axis.X<0.999 || abs t.Placement.Rotation.Z.Z<0.999
                else abs axis.Z<0.999)) then
            Error(error "unsupported-component-orientation" ["Mounts require transverse hardware sockets or axis-aligned motor side sockets."])
        elif request.Drive.Root<>request.Program.Motor.Body
             || (if compound then request.Drive.Gears.Length<driveBodies.Count
                                  || (request.Drive.Gears |> List.countBy fst |> List.exists (fun (_,count) -> count>2))
                 else request.Drive.Gears.Length<>driveBodies.Count)
             || request.Drive.Axles.Length<>driveBodies.Count
             || (request.Drive.Gears |> List.map fst |> Set.ofList)<>driveBodies
             || (request.Drive.Axles |> List.map fst |> Set.ofList)<>driveBodies
             || stockIds.Length<>(Set.ofList stockIds).Count then
            Error(error "incomplete-drive-stock" ["Each driven body needs exactly one fresh gear and axle; IDs and root must match the declared drive."])
        elif (request.Drive.Gears @ request.Drive.Axles) |> List.exists (fun (_,p) ->
            not(Vec3.finite p.Position && Rotation.valid p.Rotation))
             || request.Drive.Gears |> List.exists (fun (_,p) -> (Catalog.gearTeeth p.Part).IsNone)
             || request.Drive.Axles |> List.exists (fun (_,p) -> (Catalog.get p.Part).Connector<>Some Axle || abs p.Rotation.X.Z<0.999) then
            Error(error "invalid-drive-stock" ["Proper transverse axles and supported spur gear placements are required."])
        elif request.ObsoletePartIds.Length<>(Set.ofList request.ObsoletePartIds).Count
             || request.ObsoletePartIds |> List.exists (known.Contains >> not) then
            Error(error "invalid-obsolete-parts" ["Obsolete parts must be distinct, known source parts."])
        elif request.Reference.Feet<>request.Program.Feet
             || not(Double.IsFinite request.Drive.MotorClockScale)
             || not([1.0;5.0] |> List.exists (fun scale -> abs(abs request.Drive.MotorClockScale-scale)<1e-9)) then
            Error(error "changed-walking-requirements" ["Walking outputs and a supported unit or 5:1 reference clock must be preserved."])
        else Ok ()

    let internal validate request = validateDrive true request

    let internal scaffold (request: WalkerPackagingRequest) =
        let ground = request.Program.Bodies |> List.find _.Grounded |> _.Id
        let sourceFrame = request.Source.Bodies |> List.find (fun b -> b.Body=ground)
        let hardware = sourceFrame.Parts |> List.filter (fun p ->
            List.contains p.Id ["payload-battery";"payload-receiver";request.Source.MotorPart])
        let engaged (pin: WalkerPlacement) (part: WalkerPlacement) =
            ports part |> List.exists (fun (port,point,axis) ->
                let delta = Vec3.subtract point pin.Position
                let low,high = Catalog.shaftRange pin.Part
                abs(Vec3.dot axis pin.Rotation.X)>1.0-1e-8
                && Vec3.length(Vec3.cross delta pin.Rotation.X)<1e-6
                && Vec3.dot delta pin.Rotation.X>=low-1e-6 && Vec3.dot delta pin.Rotation.X<=high+1e-6
                && port.Kind=RoundHole)
        let hardwareMounts = request.Source.Mounts |> List.filter (fun p -> hardware |> List.exists (engaged p))
        let remove =
            request.ObsoletePartIds @ (hardwareMounts |> List.map _.Id)
            @ (sourceFrame.Parts |> List.filter (fun p -> p.Id.StartsWith "battery-support-" || p.Id="motor-spacer" || p.Id="motor-bracket") |> List.map _.Id)
            |> Set.ofList
        let frame = sourceFrame.Parts |> List.filter (fun p ->
            not(remove.Contains p.Id) && not(hardware |> List.exists (fun h -> h.Id=p.Id)))
        frame,remove

    let internal motorSupportPlane (motor: WalkerPlacement) =
        motor.Position.Z-10.0*motor.Rotation.Z.Z

    let internal motorOutputAllows (motor: WalkerPlacement) minimum maximum =
        if motor.Rotation.Z.Z>0.0 then maximum<=motor.Position.Z+20.0+1e-6
        else minimum>=motor.Position.Z-20.0-1e-6

    let internal motionSpace (request: WalkerPackagingRequest) screened =
        let _,remove = scaffold request
        let genome =
            {request.Source with
                Bodies=request.Program.Bodies |> List.map (fun body ->
                    let inherited = request.Source.Bodies |> List.tryFind (fun b -> b.Body=body.Id)
                                    |> Option.map _.Parts |> Option.defaultValue []
                    {Body=body.Id
                     Parts=(inherited |> List.filter (fun p -> not(remove.Contains p.Id) && (Catalog.gearTeeth p.Part).IsNone))
                           @ (request.Drive.Gears |> List.filter (fun (id,_) -> id=body.Id) |> List.map snd)})
                Connections=request.Source.Connections |> List.filter (fun c -> not(remove.Contains c.Placement.Id))}
        WalkerMotionSpace.reserveWithGround request.GroundClearance screened genome

    let rebuildRequestWithFeet pairedPlateFeet (s: ScreenedMotion) (source: WalkerGenome) =
        let g,footGates =
            if pairedPlateFeet then
                WalkerFootPlates.prepare s source |> Result.defaultWith (Realization.json >> invalidArg "feet")
            else source,[{Gate="foot-construction";Outcome="pass";Code="explicit-retained-feet"
                          Diagnostics=["Existing foot construction retained by explicit caller policy."]}]
        let ground = s.Program.Bodies |> List.find _.Grounded |> _.Id
        let gears = g.Bodies |> List.collect (fun b ->
            b.Parts |> List.filter (fun p -> (Catalog.gearTeeth p.Part).IsSome) |> List.map (fun p -> b.Body,p))
        let bearings = g.Connections |> List.filter (fun c ->
            (Catalog.get c.Placement.Part).Connector=Some Axle &&
            (c.Hinges |> List.exists (fun id ->
                s.Program.Hinges |> List.exists (fun h -> h.Id=id && (h.A.Body=ground || h.B.Body=ground)))))
        let axles = bearings |> List.map (fun c ->
            let h = s.Program.Hinges |> List.find (fun h -> List.contains h.Id c.Hinges)
            (if h.A.Body=ground then h.B.Body else h.A.Body),c.Placement)
        let all = (g.Bodies |> List.collect _.Parts) @ g.Mounts @ (g.Connections |> List.map _.Placement)
        let obsolete =
            (gears |> List.map (snd >> _.Id)) @ (bearings |> List.map (fun c -> c.Placement.Id))
            @ (all |> List.filter (fun p -> p.Id.StartsWith "packaging-" || p.Id.StartsWith "motor-brace-") |> List.map _.Id)
            |> List.distinct
        {Reference=s.Program;Program=s.Program;Source=g
         Targets=g.Bodies |> List.collect _.Parts |> List.filter (fun p ->
            List.contains p.Id ["payload-battery";"payload-receiver";g.MotorPart])
                 |> List.map (fun p -> {Placement=p;ToleranceLdu=if p.Id=g.MotorPart then Vec3.create 0 0 240 else Vec3.create 240 240 240})
         Drive={Root=s.Program.Motor.Body;Gears=gears;Axles=axles;MotorClockScale=1.0;InternalCompoundBearings=false}
         ObsoletePartIds=obsolete;GroundClearance=Some WalkerMotionSpace.defaultGroundClearance
         FootConstructionGates=footGates;Balance=None}

    let rebuildRequest s g = rebuildRequestWithFeet true s g

    let private build orientation (request: WalkerPackagingRequest)
                      (cachedMotion: (ScreenedMotion * ScreenedMotion * Result<MotionFrame list,string>) option)
                      (routeCache: Dictionary<string,(WalkerPlacement list * WalkerPlacement list) list> option)
                      (spaceCache: Dictionary<string,Result<WalkerMotionSpace.Space,GateDecision>> option)
                      (choose: (string -> string -> string list -> WalkerPlacement list list -> int option) option)
                      (inspect: ((unit -> WalkerConstructionInspection) -> unit) option) =
        let mutable pending = None
        let mutable activeGoal,activeRule = "hardware","reserve-gait-and-place-hardware"
        let mutable checkedParts,rejectedParts,cacheHits = 0,0,0
        let select key rule bodies alternatives =
            match choose with
            | None -> Some 0
            | Some choose ->
                let selected = choose key rule bodies alternatives
                if selected.IsNone then pending <- Some key
                selected
        let bind result next = Result.bind next result
        validateDrive choose.IsSome request |> fun valid -> bind valid (fun () ->
            let screened =
                match cachedMotion with
                | Some(reference,screened,_) -> Ok reference,Ok screened
                | None -> Realization.screen request.Reference 72,Realization.screen request.Program 72
            match screened with
            | Error gate,_ | _,Error gate -> Error gate
            | Ok reference,Ok screened ->
                let graph = screened.Graph
                let frames =
                    match cachedMotion with
                    | Some(_,_,frames) -> frames
                    | None -> Motion.sample graph (reference.Frames |> List.map (fun f -> f.InputAngle*request.Drive.MotorClockScale))
                match frames with
                | Error message -> Error(error "drive-motion-solve" [message])
                | Ok frames ->
                    let shaftIds = request.Reference.Motor.Body::(request.Reference.Transmissions |> List.collect (fun t -> [t.Input;t.Output])) |> Set.ofList
                    let retained = request.Reference.Bodies |> List.filter (fun body ->
                        not body.Grounded && (not(shaftIds.Contains body.Id) || (request.Program.Bodies |> List.exists (fun b -> b.Id=body.Id))))
                    let changed = List.zip reference.Frames frames |> List.exists (fun (a,b) ->
                        retained |> List.exists (fun body ->
                            let pa,pb = a.Poses[body.Id],b.Poses[body.Id]
                            Vec3.distance pa.Origin pb.Origin>1e-5 || abs(atan2 (sin(pa.Angle-pb.Angle)) (cos(pa.Angle-pb.Angle)))>1e-6))
                    if changed then Error(error "changed-walking-motion" ["A retained body's motion differs against the original clock."])
                    else
                        let ground = request.Program.Bodies |> List.find _.Grounded |> _.Id
                        let frameParts,remove = scaffold request
                        let anchors = frameParts |> List.collect (fun p ->
                            ports p |> List.choose (fun (port,point,axis) ->
                                if port.Kind=RoundHole && abs axis.Z>0.999 then Some point else None)) |> List.distinct
                        let addedParts,addedPins,derivation = ResizeArray<_>(),ResizeArray<_>(),ResizeArray<_>()
                        let mutable failure = None
                        let mutable reservation =
                            let key = Realization.json (request.Source,request.Drive,request.ObsoletePartIds,request.GroundClearance)
                            let space =
                                match spaceCache with
                                | Some cache ->
                                    match cache.TryGetValue key with
                                    | true,space -> space
                                    | _ ->
                                        let space = motionSpace request screened
                                        cache.Add(key,space)
                                        space
                                | None -> motionSpace request screened
                            match space with
                            | Ok space -> Some space
                            | Error gate -> failure<-Some gate;None
                        let hardware = request.Targets |> List.map _.Placement
                        let conflicts owner parts =
                            WalkerMotionSpace.fixedConflicts parts hardware
                            @ (reservation |> Option.map (fun space -> WalkerMotionSpace.conflicts space owner parts) |> Option.defaultValue [])
                        let fits owner parts =
                            let accepted =
                                WalkerMotionSpace.fitsFixed parts hardware
                                && (reservation |> Option.exists (fun space -> WalkerMotionSpace.fits space owner parts))
                            checkedParts <- checkedParts+1
                            if not accepted then rejectedParts <- rejectedParts+1
                            inspect |> Option.iter (fun report -> report (fun () ->
                                let hits =
                                    if accepted then []
                                    else
                                        (WalkerMotionSpace.fixedConflicts parts hardware |> List.truncate 4)
                                        @ (reservation |> Option.map (fun space -> WalkerMotionSpace.firstConflicts 4 space owner parts)
                                                       |> Option.defaultValue [])
                                {Goal=activeGoal;Rule=activeRule
                                 Fixed=(frameParts @ hardware @ List.ofSeq addedParts @ List.ofSeq addedPins)
                                       |> List.distinctBy _.Id |> List.map WalkerMotionSpace.partEnvelope
                                 Candidate=parts |> List.map WalkerMotionSpace.partEnvelope
                                 Reserved=reservation |> Option.map WalkerMotionSpace.envelopes |> Option.defaultValue []
                                 Conflicts=hits
                                 Checked=checkedParts;Rejected=rejectedParts;CacheHits=cacheHits}))
                            accepted
                        if failure.IsNone && not(fits ground (frameParts @ (request.Targets |> List.map _.Placement))) then
                            let conflicts = conflicts ground (frameParts @ hardware)
                            failure<-Some(error "hardware-enters-reserved-motion-space"
                                (conflicts |> List.truncate 16 |> List.map (fun c ->
                                    $"{c.Part} / {c.Obstacle} at input {c.InputAngleRadians:G8} rad")))
                        let motor = request.Targets |> List.find (fun t -> t.Placement.Id=request.Source.MotorPart) |> _.Placement
                        activeGoal <- "motor-mount"
                        activeRule <- "select-stock-motor-mount-motif"
                        let motorOptions = motorMountOptions motor |> List.filter (fun (_,parts) -> fits ground parts)
                        let motorMounts =
                            if failure.IsSome then []
                            elif motorOptions.IsEmpty then
                                failure<-Some(error "no-motion-clear-motor-mount"
                                    ["Front-forward, front-rearward and rear-saddle motor brackets all enter reserved gait or fixed hardware envelopes."])
                                []
                            else
                                let alternatives = motorOptions |> List.map snd
                                match select "motor-mount" "select-stock-motor-mount-motif" [ground] alternatives with
                                | None -> []
                                | Some index ->
                                    derivation.Add $"MotorMount -> {fst motorOptions[index]}"
                                    snd motorOptions[index]
                        let targetSockets = [
                            for target in request.Targets do
                                if target.Placement.Part="58121.dat" then
                                    for corner in motorMounts |> List.filter (fun p -> p.Id.EndsWith "-corner") do
                                        let prefix = corner.Id.Substring(0,corner.Id.Length-"-corner".Length)
                                        let mount = Vec3.add corner.Position (Rotation.apply corner.Rotation (Vec3.create 0 20 0))
                                        let pins = motorMounts |> List.filter (fun p ->
                                            p.Id.StartsWith(prefix+"-") && (Catalog.get p.Part).Connector.IsSome)
                                        let mountPin = pins |> List.find (fun p -> p.Id=prefix+"-mount-pin")
                                        let plane = 2.0*mountPin.Position.Z-mount.Z
                                        yield prefix,mount,plane,[corner],Ok pins
                                else
                                    for index,(port,socket,_) in mountPorts target.Placement |> List.indexed do
                                        let sign =
                                            if target.Placement.Id="payload-receiver" then -target.Placement.Rotation.Z.Z
                                            elif port.Position.Z<0.0 then -1.0 else 1.0
                                        let plane = socket.Z+sign*(if target.Placement.Id="payload-battery" then 40.0 else 20.0)
                                        let prefix = $"packaging-{target.Placement.Id}-{index}"
                                        yield prefix,socket,plane,[],pin (prefix+"-mount-pin") {socket with Z=plane} socket |> Result.map List.singleton
                        ]
                        let mutable occupied =
                            (request.Source.Mounts |> List.filter (fun p -> not(remove.Contains p.Id)))
                            @ (request.Source.Connections |> List.map _.Placement |> List.filter (fun p -> not(remove.Contains p.Id)))
                            @ (request.Drive.Axles |> List.map (fun (_,p) -> {p with Part="3708.dat"}))
                            @ (targetSockets |> List.collect (fun (_,_,_,_,pins) -> match pins with Ok pins -> pins | Error _ -> []))
                        let obstacles = [
                            for _,gear in request.Drive.Gears do
                                yield WalkerClearance.initialEnvelope gear
                            for body in request.Source.Bodies do
                                if body.Body<>ground && frames.Head.Poses.ContainsKey body.Body then
                                    for part in body.Parts do
                                        if not(remove.Contains part.Id) then
                                            yield WalkerClearance.sweptEnvelope screened.Frames.Head frames body.Body part
                            for connector in request.Source.Connections do
                                if not(remove.Contains connector.Placement.Id) then
                                    let hinge = request.Program.Hinges |> List.find (fun h -> h.Id=connector.Hinges.Head)
                                    let body = if hinge.A.Body=ground then hinge.B.Body else hinge.A.Body
                                    yield WalkerClearance.sweptEnvelope screened.Frames.Head frames body connector.Placement
                        ]
                        let mutable shaftObstacles = request.Drive.Axles |> List.map (fun (body,part) ->
                            body,WalkerClearance.initialEnvelope {part with Part="3708.dat"})
                        let risk bodies parts =
                            let envelopes = obstacles @ (shaftObstacles |> List.choose (fun (body,envelope) ->
                                if List.contains body bodies then None else Some envelope))
                            parts |> List.sumBy (fun part ->
                                let envelope = WalkerClearance.initialEnvelope part
                                envelopes |> List.sumBy (WalkerClearance.envelopeOverlapRisk envelope))
                        let addSupport prefix socket plane extras mounts bodies =
                            if failure.IsNone && pending.IsNone then
                                activeGoal <- prefix
                                activeRule <- "stock-beam-route-outside-reserved-gait"
                                let routeFailure () =
                                    let first =
                                        motionSupportRoutes (fun _ -> true) orientation prefix anchors
                                            (occupied @ List.ofSeq addedPins) socket plane |> List.tryHead
                                    let blocked =
                                        first |> Option.map (fun (parts,pins) ->
                                            conflicts ground (parts @ extras @ pins @ mounts)
                                            |> List.truncate 4 |> List.map (fun c ->
                                                $"{c.Part} / {c.Obstacle} at input {c.InputAngleRadians:G8} rad"))
                                        |> Option.defaultValue []
                                    error "no-motion-clear-support-route" (prefix::blocked)
                                let routes =
                                    match choose with
                                    | None ->
                                        motionSupportRoutesWithPruning (fun (parts,pins) -> fits ground (parts @ extras @ pins @ mounts))
                                            orientation prefix anchors (occupied @ List.ofSeq addedPins) socket plane
                                        |> List.tryHead |> Option.map (List.singleton >> Ok)
                                        |> Option.defaultWith (fun () -> Error(routeFailure ()))
                                    | Some _ ->
                                        let used = occupied @ List.ofSeq addedPins
                                        let cacheKey = Realization.json (prefix,anchors,used,obstacles,shaftObstacles,bodies,socket,plane,request.GroundClearance)
                                        let choices () =
                                            [1;-1] |> List.collect (fun side ->
                                                motionSupportRoutesWithPruning (fun (parts,pins) -> fits ground (parts @ extras @ pins @ mounts))
                                                    side prefix anchors used socket plane)
                                            |> List.sortBy (fun (parts,pins) -> risk bodies (parts @ pins),parts |> List.sumBy (fun p ->
                                                (Catalog.get p.Part).Ports.Length))
                                            |> List.truncate 8
                                        let choices =
                                            match routeCache with
                                            | None -> choices ()
                                            | Some cache ->
                                                match cache.TryGetValue cacheKey with
                                                | true,choices -> cacheHits<-cacheHits+1;choices
                                                | _ ->
                                                    let routes = choices ()
                                                    cache.Add(cacheKey,routes)
                                                    routes
                                        if choices.IsEmpty then Error(routeFailure ()) else Ok choices
                                match routes with
                                | Error gate -> failure<-Some gate
                                | Ok routes ->
                                    let basic = routes |> List.map (fun (parts,pins) -> parts @ extras @ pins @ mounts)
                                    let alternatives =
                                        match choose with
                                        | None -> basic
                                        | Some _ ->
                                            let longer =
                                                basic |> List.collect (fun parts ->
                                                    let short = parts |> List.filter (fun p -> p.Part="4459.dat") |> List.map _.Id
                                                    (short |> List.map List.singleton) @ (if short.Length>1 then [short] else [])
                                                    |> List.map (fun ids -> parts |> List.map (fun p ->
                                                        if List.contains p.Id ids then {p with Part="6558.dat"} else p)))
                                                |> List.filter (fun parts ->
                                                    let pins = parts |> List.filter (fun p -> (Catalog.get p.Part).Connector.IsSome)
                                                    pins |> List.forall (fun pin ->
                                                        occupied @ List.ofSeq addedPins @ pins |> List.forall (fun other ->
                                                            pin.Id=other.Id ||
                                                            not(Catalog.shaftsOverlap pin.Part pin.Position pin.Rotation
                                                                 other.Part other.Position other.Rotation))))
                                            basic @ (longer |> List.distinct |> List.truncate 8)
                                    let structural = frameParts @ (request.Targets |> List.map _.Placement) @ List.ofSeq addedParts
                                    let alternatives =
                                        alternatives |> List.filter (fits ground) |> List.sortBy (fun parts ->
                                            let beams = parts |> List.filter (fun p -> (Catalog.get p.Part).Connector.IsNone)
                                            let pins = parts |> List.filter (fun p -> (Catalog.get p.Part).Connector.IsSome)
                                            pins |> List.sumBy (fun p -> contactCapacityDeficit p (structural @ beams) 0),
                                            risk bodies parts)
                                    let rule = if (fst routes.Head).Length=2 then "two-beam-covered-pin-bracket" else "multi-beam-covered-pin-bracket"
                                    match select prefix rule bodies alternatives with
                                    | None -> ()
                                    | Some index ->
                                        let pins,parts = alternatives[index] |> List.partition (fun p -> (Catalog.get p.Part).Connector.IsSome)
                                        addedParts.AddRange parts;addedPins.AddRange pins
                                        derivation.Add $"Support({prefix}) -> {rule}"
                        let gearMap = request.Drive.Gears |> List.groupBy fst |> List.map (fun (body,gears) -> body,gears |> List.map snd) |> Map.ofList
                        let axleMap = request.Drive.Axles |> Map.ofList
                        let driveBodies = request.Program.Motor.Body::(request.Program.Transmissions |> List.collect (fun t -> [t.Input;t.Output])) |> Set.ofList
                        let bearings = request.Program.Hinges |> List.filter (fun h ->
                            (h.A.Body=ground && driveBodies.Contains h.B.Body) || (h.B.Body=ground && driveBodies.Contains h.A.Body))
                        let bearingChoices = ResizeArray<_>()
                        let bearingSupports = ResizeArray<_>()
                        for hinge in bearings do
                            let body = if hinge.A.Body=ground then hinge.B.Body else hinge.A.Body
                            if pending.IsSome || failure.IsSome then ()
                            elif not(axleMap.ContainsKey body && gearMap.ContainsKey body) then
                                failure<-Some(error "incomplete-drive-stock" [$"Missing gear or axle for {body}."])
                            else
                                let requestedAxle,gears = axleMap[body],gearMap[body]
                                let gear = gears.Head
                                let ownedParts = request.Source.Bodies |> List.tryFind (fun b -> b.Body=body)
                                                 |> Option.map _.Parts |> Option.defaultValue []
                                                 |> List.filter (fun p -> not(remove.Contains p.Id))
                                let socketWalls = ownedParts |> List.collect (fun p ->
                                    ports p |> List.choose (fun (port,point,axis) ->
                                        if port.Kind=AxleHole && abs axis.Z>0.999
                                           && abs(point.X-gear.Position.X)<1e-5 && abs(point.Y-gear.Position.Y)<1e-5 then
                                            let half = Catalog.socketHalfThickness p.Part
                                            Some [point.Z-half;point.Z+half]
                                        else None) |> List.concat)
                                let motorSockets =
                                    if body=request.Drive.Root then
                                        request.Targets |> List.find (fun t -> t.Placement.Id=request.Source.MotorPart)
                                        |> _.Placement |> ports |> List.choose (fun (port,point,_) ->
                                            if port.Kind=AxleHole then Some point.Z else None)
                                    else []
                                let planes = driveBearingPlanes request body gears
                                let wallDepths = (gears |> List.collect (fun gear -> [gear.Position.Z-10.0;gear.Position.Z+10.0]))
                                                @ (planes |> List.collect (fun z -> [z-10.0;z+10.0]))
                                                @ socketWalls
                                let minimum = List.min (wallDepths @ motorSockets)
                                let maximum = List.max (wallDepths @ motorSockets)
                                let motorTarget = request.Targets |> List.find (fun t -> t.Placement.Id=request.Source.MotorPart) |> _.Placement
                                if body=request.Drive.Root && not(motorOutputAllows motorTarget minimum maximum) then
                                    failure<-Some(error "motor-shaft-through-housing" ["Required bearing/load sockets extend past the supported motor output channel."])
                                let span = maximum-minimum
                                let stocks = Catalog.axleParts |> List.filter (fun part ->
                                    let low,high = Catalog.shaftRange part
                                    high-low>=span-1e-6)
                                let spanningAxles = stocks |> List.collect (fun part ->
                                    let low,high = Catalog.shaftRange part
                                    let centers =
                                        if body=request.Drive.Root && motorTarget.Rotation.Z.Z>0.0 then [maximum-high]
                                        elif body=request.Drive.Root then [minimum-low]
                                        elif choose.IsNone then [(minimum+maximum-low-high)/2.0]
                                        else [(minimum+maximum-low-high)/2.0;maximum-high;minimum-low] |> List.distinct
                                    centers |> List.map (fun center ->
                                        {requestedAxle with Id="pivot-"+hinge.Id;Part=part;Position={requestedAxle.Position with Z=center}}))
                                activeGoal <- "axle-"+body
                                activeRule <- "stock-axle-wall-coverage"
                                let axleChoices = spanningAxles |> List.filter (fun axle -> fits body [axle]) |> List.sortBy (fun axle ->
                                        let foreign =
                                            request.Source.Bodies |> List.filter (fun b -> b.Body<>body && b.Body<>ground)
                                            |> List.collect _.Parts |> List.filter (fun p -> not(remove.Contains p.Id))
                                            |> List.filter (WalkerStructural.engaged axle) |> List.length
                                        let structural = gears @ ownedParts @ frameParts @ List.ofSeq addedParts
                                                         @ (request.Targets |> List.map _.Placement)
                                        let low,high = Catalog.shaftRange axle.Part
                                        foreign,contactCapacityDeficit axle structural planes.Length,high-low)
                                if failure.IsNone && axleChoices.IsEmpty then
                                    let conflicts =
                                        spanningAxles |> List.tryHead |> Option.map (fun axle ->
                                            conflicts body [axle]
                                            |> List.truncate 4 |> List.map (fun c -> $"{c.Part} / {c.Obstacle} at input {c.InputAngleRadians:G8} rad"))
                                        |> Option.defaultValue []
                                    failure<-Some(error (if spanningAxles.IsEmpty then "unsupported-output-axle-span" else "no-motion-clear-output-axle")
                                        ($"Required bearing/load socket wall span for {body} is {span:G6} LDU."::conflicts))
                                if failure.IsNone then
                                    match select ("axle-"+body) "stock-axle-wall-coverage" [body] (axleChoices |> List.map List.singleton) with
                                    | None -> ()
                                    | Some index ->
                                        let axle = axleChoices[index]
                                        let low,high = Catalog.shaftRange axle.Part
                                        for index,plane in List.indexed planes do
                                            if plane-10.0<axle.Position.Z+low-1e-6 || plane+10.0>axle.Position.Z+high+1e-6 then
                                                failure<-Some(error "insufficient-bearing-wall-reach" [$"Axle {body} does not cover the new bearing wall at {plane:G6}."])
                                            else bearingSupports.Add($"packaging-bearing-{body}-{index}",{gear.Position with Z=plane},plane,body,axle)
                                        if pending.IsNone then
                                            bearingChoices.Add {Group=hinge.Id;Hinges=[hinge.Id];Placement=axle}
                        if pending.IsNone && failure.IsNone then
                            let axes = bearingSupports |> Seq.map (fun (_,_,_,body,axle) -> body,axle) |> Seq.distinct |> Seq.toList
                            reservation <- reservation |> Option.map (fun space ->
                                axes |> List.fold (fun current (body,axle) -> WalkerMotionSpace.addBearingAxis current body axle) space)
                            occupied <- (request.Source.Mounts |> List.filter (fun p -> not(remove.Contains p.Id)))
                                        @ (request.Source.Connections |> List.map _.Placement |> List.filter (fun p -> not(remove.Contains p.Id)))
                                        @ (axes |> List.map snd)
                                        @ (targetSockets |> List.collect (fun (_,_,_,_,pins) -> match pins with Ok pins -> pins | Error _ -> []))
                            shaftObstacles <- axes |> List.map (fun (body,axle) -> body,WalkerClearance.initialEnvelope axle)
                            for prefix,socket,plane,extras,mounts in targetSockets do
                                match mounts with
                                | Error gate -> failure<-Some gate
                                | Ok mounts -> addSupport prefix socket plane extras mounts [ground]
                            for prefix,socket,plane,body,_ in bearingSupports do
                                addSupport prefix socket plane [] [] [body;ground]
                        match pending,failure with
                        | Some key,_ -> Error(error "construction-incomplete" [key])
                        | _,Some gate -> Error gate
                        | None,None ->
                            let bearingIds = bearings |> List.map _.Id |> Set.ofList
                            let bodies = request.Program.Bodies |> List.map (fun body ->
                                let inherited = request.Source.Bodies |> List.tryFind (fun b -> b.Body=body.Id)
                                let parts =
                                    if body.Grounded then frameParts @ (request.Targets |> List.map _.Placement) @ List.ofSeq addedParts
                                    else
                                        (inherited |> Option.map _.Parts |> Option.defaultValue [] |> List.filter (fun p -> not(remove.Contains p.Id)))
                                        @ (gearMap |> Map.tryFind body.Id |> Option.defaultValue [])
                                {Body=body.Id;Parts=parts})
                            let genome = {request.Source with MechanismId=screened.MechanismId;Bodies=bodies
                                                              Speed=request.Source.Speed*request.Drive.MotorClockScale
                                                              Connections=(request.Source.Connections |> List.filter (fun c ->
                                                                  not(remove.Contains c.Placement.Id) && not(c.Hinges |> List.exists bearingIds.Contains)))
                                                                          @ List.ofSeq bearingChoices
                                                              Mounts=(request.Source.Mounts |> List.filter (fun p -> not(remove.Contains p.Id))) @ List.ofSeq addedPins}
                            match WalkerRealization.compile screened genome with
                            | Error gate -> Error gate
                            | Ok realized ->
                                let geometry = Realization.screenAssemblyGeometry screened realized.Assembly
                                let ground = groundConflicts request screened realized.Genome
                                if not ground.IsEmpty then
                                    Error(error "planned-nonfoot-ground-clearance"
                                        (ground |> List.truncate 16 |> List.map (fun c ->
                                            $"{c.Part} / {c.Obstacle} at input {c.InputAngleRadians:G8} rad")))
                                elif geometry.Outcome="reject" then Error geometry
                                else
                                    let clearance =
                                        if choose.IsSome then WalkerClearance.evaluateFrames screened frames realized.Genome 0
                                        else WalkerClearance.inspect screened realized.Genome |> snd
                                    Ok {Screened=screened;Realized=realized
                                        AddedPartIds=(List.ofSeq addedParts @ List.ofSeq addedPins
                                                      @ (request.Drive.Gears |> List.map snd)
                                                      @ (List.ofSeq bearingChoices |> List.map _.Placement) |> List.map _.Id)
                                        RetainedFramePartIds=frameParts |> List.map _.Id;RemovedPartIds=Set.toList remove
                                        Derivation=["Assembly -> RetainedGroundAnchors + TargetMounts + DriveBearings + WalkingLinks"]
                                                   @ List.ofSeq derivation
                                        Clearance=clearance})

    let internal replayObserved inspect request cachedMotion routeCache spaceCache choose =
        build 1 request (Some cachedMotion) (Some routeCache) (Some spaceCache) (Some choose) inspect

    let internal replay request cachedMotion routeCache spaceCache choose =
        replayObserved None request cachedMotion routeCache spaceCache choose

    let private construct orientation request = build orientation request None None None None None

    let realize request limit =
        if limit<1 || limit>maximumAttempts then invalidArg "limit" "Packaging realization permits one or two support orientations."
        [for orientation in [1;-1] |> List.truncate limit ->
            match construct orientation request with
            | Ok construction ->
                {Orientation=orientation;Construction=Some construction
                 Gates=[{Gate="packaging-realization";Outcome="pass";Code="bounded-target-supports-and-drive-bearings"
                         Diagnostics=["Complete compiler-accepted assembly; exact reconstruction and load checks remain mandatory."]}]}
            | Error gate -> {Orientation=orientation;Construction=None;Gates=[gate]}]

    let repair (request: WalkerPackagingRequest) (construction: WalkerPackagingConstruction)
               (feedback: WalkerConstructionFeedback) limit =
        if limit<1 || limit>32 then invalidArg "limit" "Packaging repair permits one to 32 construction alternatives."
        let parts = construction.Realized.Assembly.Parts |> List.map _.Id |> Set.ofList
        if feedback.Parts.IsEmpty || feedback.Parts |> List.exists (parts.Contains >> not) then
            invalidArg "feedback" "Repair feedback must identify existing assembly parts."
        let choices = WalkerConstruction.repairs construction.Screened construction.Realized.Genome feedback limit
        let targetFits (genome: WalkerGenome) =
            let all = genome.Bodies |> List.collect _.Parts |> List.map (fun p -> p.Id,p) |> Map.ofList
            request.Targets |> List.forall (fun target ->
                match all |> Map.tryFind target.Placement.Id with
                | None -> false
                | Some part ->
                    let delta = Vec3.subtract part.Position target.Placement.Position
                    part.Part=target.Placement.Part && part.Rotation=target.Placement.Rotation
                    && abs delta.X<=target.ToleranceLdu.X+1e-6 && abs delta.Y<=target.ToleranceLdu.Y+1e-6
                    && abs delta.Z<=target.ToleranceLdu.Z+1e-6)
        choices |> List.filter (fun choice -> targetFits choice.Genome)
        |> List.choose (fun choice ->
            match WalkerRealization.compile construction.Screened choice.Genome with
            | Error _ -> None
            | Ok realized ->
                let geometry = Realization.screenAssemblyGeometry construction.Screened realized.Assembly
                if geometry.Outcome="reject" || not((groundConflicts request construction.Screened realized.Genome).IsEmpty) then None
                else
                    let _,clearance = WalkerClearance.inspect construction.Screened realized.Genome
                    Some {construction with Realized=realized;Clearance=clearance
                                            Derivation=construction.Derivation @ [$"Feedback({feedback.Kind}) -> {choice.Motif}"]})
