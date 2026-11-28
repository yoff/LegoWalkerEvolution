namespace LegoWalkers

open LegoWalkers
open Xunit

module WalkerPackagingTests =
    let private fixture () =
        let program,genome = WalkerSeeds.fourBar 2
        let unwrap result = Result.defaultWith (Realization.json >> failwith) result
        let screened = Realization.screen program 72 |> unwrap
        let realized = WalkerRealization.compile screened genome |> unwrap
        let shafts = program.Motor.Body::(program.Transmissions |> List.collect (fun t -> [t.Input;t.Output])) |> Set.ofList
        let gears = genome.Bodies |> List.collect (fun body ->
            body.Parts |> List.filter (fun p -> (Catalog.gearTeeth p.Part).IsSome)
            |> List.map (fun p -> body.Body,{p with Id="fresh-"+p.Id}))
        let bearings = genome.Connections |> List.filter (fun c -> (Catalog.get c.Placement.Part).Connector=Some Axle)
        let axles = bearings |> List.map (fun c ->
            let hinge = program.Hinges |> List.find (fun h -> h.Id=c.Hinges.Head)
            let body = if hinge.A.Body=realized.Assembly.Chassis then hinge.B.Body else hinge.A.Body
            body,{c.Placement with Id="fresh-"+c.Placement.Id})
        let removed =
            (genome.Bodies |> List.collect (fun b ->
                if shafts.Contains b.Body then
                    b.Parts |> List.filter (fun p -> (Catalog.gearTeeth p.Part).IsSome || p.Part="3713.dat" || p.Part="4265c.dat")
                    |> List.map _.Id
                else []))
            @ (bearings |> List.map (fun c -> c.Placement.Id))
        {Reference=program;Program=program;Source=genome
         Targets=genome.Bodies |> List.collect _.Parts |> List.filter (fun p ->
            List.contains p.Id ["payload-battery";"payload-receiver";genome.MotorPart])
                 |> List.map (fun p ->
                     let placed =
                         if p.Id=genome.MotorPart then {p with Position={p.Position with Z=p.Position.Z+20.0}}
                         elif p.Id="payload-receiver" then {p with Position={p.Position with Y=p.Position.Y-80.0}}
                         else p
                     {Placement=placed;ToleranceLdu=Vec3.zero})
         Drive={Root=program.Motor.Body;Gears=gears;Axles=axles;MotorClockScale=1.0;InternalCompoundBearings=false}
         ObsoletePartIds=removed;GroundClearance=None;FootConstructionGates=[];Balance=None}

    [<Fact>]
    let ``motor layout choices reroot only same signed rate shafts and preserve solved gait`` () =
        let program,genome = WalkerSeeds.fourBar 4
        let unwrap result = result |> Result.defaultWith (Realization.json >> failwith)
        let screened = Realization.screen program 72 |> unwrap
        let request = WalkerPackaging.rebuildRequest screened genome
        let layouts = WalkerConstructionTheory.motorLayoutOptions request
        Assert.Contains(request,layouts)
        Assert.True(layouts |> List.exists (fun layout -> layout.Drive.Root<>request.Drive.Root))
        Assert.Contains(layouts,fun layout ->
            (layout.Targets |> List.find (fun target -> target.Placement.Id=genome.MotorPart)).Placement.Rotation.Z.Z<0.0)
        for layout in layouts do
            Assert.Equal(request.Reference,layout.Reference)
            Assert.Equal(request.Source.Speed,layout.Source.Speed)
            Assert.Equal(request.Source.MaxTorque,layout.Source.MaxTorque)
            Assert.Equal(1.0,layout.Drive.MotorClockScale)
            Assert.Equal(Realization.mechanismId layout.Program,layout.Source.MechanismId)
            let solved = Realization.screen layout.Program 72 |> unwrap
            for before,after in List.zip screened.Frames solved.Frames do
                for body in program.Bodies do
                    Assert.True(Vec3.distance before.Poses[body.Id].Origin after.Poses[body.Id].Origin<1e-5)
                    let delta = before.Poses[body.Id].Angle-after.Poses[body.Id].Angle
                    Assert.True(abs(atan2 (sin delta) (cos delta))<1e-6)
        let targets = request.Targets |> List.map (fun target -> {target with ToleranceLdu=Vec3.zero})
        let pinned = {request with Targets=targets}
        Assert.Equal<WalkerPackagingRequest list>([pinned],WalkerConstructionTheory.motorLayoutOptions pinned)
        let gearedClock = {request with Drive={request.Drive with MotorClockScale=0.5}}
        Assert.Equal<WalkerPackagingRequest list>([gearedClock],WalkerConstructionTheory.motorLayoutOptions gearedClock)

    [<Fact>]
    let ``compatibility propagation regenerates blocked truncated pools but reuses surviving routes`` () =
        let pin id x = {Id=id;Part="4459.dat";Color=8;Position=Vec3.create x 0 0
                        Rotation={X=Vec3.create 0 0 1;Y=Vec3.create 1 0 0;Z=Vec3.create 0 1 0}}
        let occupied,blocked,clear = pin "occupied" 0,pin "blocked" 0,pin "clear" 40
        let mutable regenerations = 0
        let regenerate routes () = regenerations<-regenerations+1;routes
        Assert.True(WalkerPackaging.supportDomainCompatible [occupied] [[],[clear]] [] (regenerate []))
        Assert.Equal(0,regenerations)
        Assert.True(WalkerPackaging.supportDomainCompatible [occupied] [[],[blocked]] [] (regenerate [[],[clear]]))
        Assert.Equal(1,regenerations)
        Assert.False(WalkerPackaging.supportDomainCompatible [occupied] [[],[blocked]] [] (regenerate []))
        Assert.Equal(2,regenerations)
        Assert.False(WalkerPackaging.supportDomainCompatible [occupied] [[],[clear]] [blocked] (regenerate [[],[clear]]))
        Assert.Equal(2,regenerations)

    [<Fact>]
    let ``route inspection exposes constraint geometry while preserving bounded search`` () =
        let request = fixture ()
        let initial = WalkerConstructionTheory.start request "inspection-oracle" 16 16
        let expected,witness = WalkerConstructionTheory.advance request "inspection-oracle" initial
        let inspections = ResizeArray<WalkerConstructionInspection>()
        let actual,observed =
            WalkerConstructionTheory.advanceObserved (fun _ _ _ -> ())
                (fun produce -> inspections.Add(produce ())) request "inspection-oracle" initial
        Assert.Equal(expected,actual)
        Assert.Equal(witness |> Option.map _.Realized.AssemblyId,observed |> Option.map _.Realized.AssemblyId)
        Assert.NotEmpty inspections
        Assert.True(inspections |> Seq.exists (fun entry -> entry.Goal.Contains "support" || entry.Rule.Contains "route"))
        Assert.True(inspections |> Seq.exists (fun entry -> not entry.Candidate.IsEmpty && not entry.Reserved.IsEmpty))
        Assert.True(inspections |> Seq.exists (fun entry -> entry.Rejected>0))

    [<Fact>]
    let ``three beam support closes axial reach using legal covered friction pins`` () =
        let anchor,socket = Vec3.create 0 0 100,Vec3.create 120 0 210
        let routes = WalkerPackaging.supportRoutes 1 "extended" [anchor] [] socket socket.Z
        Assert.NotEmpty routes
        for parts,pins in routes do
            Assert.Equal(3,parts.Length)
            Assert.Equal(3,pins.Length)
            let sockets = anchor::(parts |> List.collect (fun part ->
                (Catalog.get part.Part).Ports |> List.map (fun port ->
                    Vec3.add part.Position (Rotation.apply part.Rotation port.Position))))
            for pin in pins do
                let low,high = Catalog.shaftRange pin.Part
                let engaged = sockets |> List.distinct |> List.filter (fun socket ->
                    let delta = Vec3.subtract socket pin.Position
                    Vec3.length(Vec3.cross delta pin.Rotation.X)<1e-6
                    && Vec3.dot delta pin.Rotation.X>=low && Vec3.dot delta pin.Rotation.X<=high)
                Assert.Equal(2,engaged.Length)
                for socket in engaged do
                    let along = Vec3.dot (Vec3.subtract socket pin.Position) pin.Rotation.X
                    Assert.True(along-10.0>=low-1e-6 && along+10.0<=high+1e-6)
                Assert.Equal(0,WalkerPackaging.contactCapacityDeficit pin parts 0)
        Assert.Empty(WalkerPackaging.supportRoutes 1 "too-far" [anchor] [] {socket with Z=320.0} 320.0)

    [<Fact>]
    let ``bounded support can turn away from an occupied anchor before reaching the bearing plane`` () =
        let anchor,socket = Vec3.create 0 0 120,Vec3.create 120 0 210
        let template = (fixture ()).Drive.Gears.Head |> snd
        let occupied = {template with
                            Part="4459.dat";Position=Vec3.create 0 0 155
                            Rotation={X=Vec3.create 0 0 1;Y=Vec3.create 1 0 0;Z=Vec3.create 0 1 0}}
        let routes = WalkerPackaging.supportRoutes 1 "around-pin" [anchor] [occupied] socket socket.Z
        Assert.NotEmpty routes
        for parts,pins in routes do
            Assert.InRange(parts.Length,4,5)
            Assert.Equal(parts.Length,pins.Length)
            for pin in pins do
                Assert.False(Catalog.shaftsOverlap pin.Part pin.Position pin.Rotation occupied.Part occupied.Position occupied.Rotation)

    [<Fact>]
    let ``extended bracket bonds include every bridge pin and mount the final beam`` () =
        let prefix = "packaging-payload-battery-0"
        let anchor,socket = Vec3.create 0 0 100,Vec3.create 120 0 210
        let parts,pins = WalkerPackaging.supportRoutes 1 prefix [anchor] [] socket socket.Z |> List.head
        let frame = {parts.Head with Id="frame-anchor";Position={parts.Head.Position with Z=anchor.Z}}
        let last = parts |> List.last
        let target = {last with Id="payload-battery";Position={last.Position with Z=230.0}}
        let mount = {pins.Head with
                        Id=prefix+"-mount-pin";Part="4459.dat";Position=Vec3.create 120 0 220}
        let bonds = WalkerConstructionTheory.bracketBonds prefix (parts @ pins @ [mount]) [frame] [target]
        Assert.Equal(pins.Length+1,bonds.Length)
        let mountBond = bonds |> List.find (fun b -> b.Connector=mount.Id)
        Assert.Equal(last.Id,mountBond.PartA)
        Assert.Equal(target.Id,mountBond.PartB)
        for pin in pins do Assert.Single(bonds |> List.filter (fun b -> b.Connector=pin.Id)) |> ignore

    [<Fact>]
    let ``bearing stack keeps existing collars instead of putting a beam through them`` () =
        let request = fixture ()
        let body,gear = request.Drive.Gears.Head
        let collar = {gear with Id="retained-stack-collar";Part="3713.dat";Position={gear.Position with Z=gear.Position.Z-20.0}}
        let source =
            {request.Source with
                Bodies=request.Source.Bodies |> List.map (fun b ->
                    if b.Body=body then {b with Parts=b.Parts @ [collar]} else b)}
        let request = {request with Source=source}
        let planes = WalkerPackaging.driveBearingPlanes request body [gear]
        Assert.Equal<float list>([gear.Position.Z-40.0;gear.Position.Z+20.0],planes)
        Assert.All(planes,fun plane -> Assert.True(abs(plane-collar.Position.Z)>=20.0-1e-6))
        Assert.Contains(collar,(request.Source.Bodies |> List.find (fun b -> b.Body=body)).Parts)

    [<Fact>]
    let ``wide reduction gear reserves clearance from bearing beam faces`` () =
        let gear = (fixture ()).Drive.Gears.Head |> snd
        let gears = [{gear with Part="3648b.dat";Position={gear.Position with Z=100.0}};
                     {gear with Id=gear.Id+"-compound";Part="3649.dat";Position={gear.Position with Z=220.0}}]
        Assert.Equal<float list>([80.0;120.0;190.0;250.0],WalkerPackaging.bearingPlanes false gears)
        Assert.Equal<float list>([120.0;190.0],WalkerPackaging.bearingPlanes true gears)
        Assert.Equal(10.0,220.0-10.0-(190.0+10.0))
        Assert.Throws<System.ArgumentException>(fun () ->
            WalkerPackaging.bearingPlanes true [gears[0];{gears[1] with Position={gears[1].Position with Z=160.0}}] |> ignore) |> ignore

    [<Fact>]
    let ``scoped motor reduction moves only the root mesh and preserves downstream gear ratio`` () =
        let request = fixture ()
        Assert.Equal(request,WalkerConstructionTheory.withMotorReduction 1 request)
        Assert.Throws<System.ArgumentException>(fun () -> WalkerConstructionTheory.withMotorReduction 2 request |> ignore) |> ignore
        let edge = request.Program.Transmissions |> List.find (fun t -> t.Input=request.Drive.Root)
        let root = request.Drive.Gears |> List.find (fun (body,_) -> body=edge.Input) |> snd
        let gears = request.Drive.Gears |> List.map (fun (body,gear) ->
            if body=edge.Input then body,{gear with Part="3648b.dat"}
            elif body=edge.Output then body,{gear with Part="3648b.dat";Position={root.Position with X=root.Position.X+60.0}}
            else body,gear)
        let request = {request with Drive={request.Drive with Gears=gears}}
        let reduced = WalkerConstructionTheory.withMotorReduction 5 request
        Assert.Equal(request.Reference,reduced.Reference)
        Assert.Equal(request.Source,reduced.Source)
        Assert.Equal<WalkerComponentTarget list>(request.Targets,reduced.Targets)
        Assert.Equal<(string * WalkerPlacement) list>(request.Drive.Axles,reduced.Drive.Axles)
        Assert.Equal(request.Drive.MotorClockScale*5.0,reduced.Drive.MotorClockScale)
        let replacement = reduced.Program.Transmissions |> List.find (fun t -> t.Input=edge.Input)
        Assert.Equal(edge.Ratio/5.0,replacement.Ratio)
        Assert.Equal<MotionTransmission list>(request.Program.Transmissions |> List.filter (fun t -> t<>edge),
                     reduced.Program.Transmissions |> List.filter (fun t -> t<>replacement))
        let reducedRoot = reduced.Drive.Gears |> List.find (fun (body,_) -> body=edge.Input) |> snd
        Assert.Equal("3647.dat",reducedRoot.Part)
        Assert.Equal(root.Position.Z+80.0,reducedRoot.Position.Z)
        let routes = WalkerConstructionTheory.driveOptions reduced @ WalkerConstructionTheory.motorDriveOptions reduced
                     |> List.distinctBy (fun route -> route.Drive)
        Assert.NotEmpty routes
        for route in routes do
            let root = route.Drive.Gears |> List.find (fun (body,_) -> body=edge.Input) |> snd
            let output = route.Drive.Gears |> List.filter (fun (body,_) -> body=edge.Output) |> List.map snd
            for gear in output do
                Assert.Equal((if abs(gear.Position.Z-root.Position.Z)<1e-6 then "3649.dat" else "3648b.dat"),gear.Part)
            for transmission in route.Program.Transmissions do
                let pairs = [for bodyA,a in route.Drive.Gears do
                                for bodyB,b in route.Drive.Gears do
                                    if bodyA=transmission.Input && bodyB=transmission.Output
                                       && Catalog.matchesSpurMesh transmission.Ratio a.Part a.Position b.Part b.Position then yield a,b]
                Assert.Single pairs |> ignore
        Assert.Throws<System.ArgumentException>(fun () -> WalkerConstructionTheory.withMotorReduction 5 reduced |> ignore) |> ignore

    [<Fact>]
    let ``axial motor alternatives respect tolerance and retain the original placement`` () =
        let request = fixture ()
        let motor = request.Targets |> List.find (fun t -> t.Placement.Id=request.Source.MotorPart)
        let placement = {motor.Placement with
                            Position=Vec3.create 0 0 120
                            Rotation={X=Vec3.create -1 0 0;Y=Vec3.create 0 1 0;Z=Vec3.create 0 0 -1}}
        let targets = request.Targets |> List.map (fun t ->
            if t.Placement.Id=motor.Placement.Id then {t with Placement=placement} else t)
        let gears = request.Drive.Gears |> List.mapi (fun index (body,part) ->
            let x = if body=request.Drive.Root then 0.0 else float(index+1)*500.0
            body,{part with Position=Vec3.create x 0 100})
        let request = {request with Targets=targets;Drive={request.Drive with Gears=gears}}
        Assert.Single(WalkerConstructionTheory.targetOptions request) |> ignore
        let request = {request with Targets=request.Targets |> List.map (fun t ->
                                        if t.Placement.Id=motor.Placement.Id then {t with ToleranceLdu=Vec3.create 0 0 60} else t)}
        let options = WalkerConstructionTheory.targetOptions request
        Assert.Contains(options,fun targets ->
            (targets |> List.find (fun t -> t.Placement.Id=motor.Placement.Id)).Placement.Position=Vec3.create 0 0 60)
        Assert.Contains(request.Targets,options)
        for targets in options do
            for target in targets do
                let original = request.Targets |> List.find (fun t -> t.Placement.Id=target.Placement.Id)
                Assert.Equal(original.Placement.Position.X,target.Placement.Position.X)
                Assert.Equal(original.Placement.Position.Y,target.Placement.Position.Y)
                Assert.True(abs(target.Placement.Position.Z-original.Placement.Position.Z)<=original.ToleranceLdu.Z)
                Assert.Equal(original.Placement.Rotation,target.Placement.Rotation)
        Assert.Equal(100.0,(request.Drive.Gears |> List.find (fun (body,_) -> body=request.Drive.Root) |> snd).Position.Z)

    [<Theory>]
    [<InlineData(1.0)>]
    [<InlineData(-1.0)>]
    let ``centered motor placement reroutes its mesh without changing shaft centers ratios or walking motion`` face =
        let request = fixture ()
        let motor = request.Targets |> List.find (fun t -> t.Placement.Id=request.Source.MotorPart)
        let placement = {motor.Placement with
                            Position={motor.Placement.Position with Z=if face>0.0 then 150.0 else -30.0}
                            Rotation={X=Vec3.create face 0 0;Y=Vec3.create 0 1 0;Z=Vec3.create 0 0 face}}
        let targets = request.Targets |> List.map (fun t ->
            if t.Placement.Id=motor.Placement.Id then {t with Placement=placement;ToleranceLdu=Vec3.create 0 0 240} else t)
        let request = {request with Targets=targets}
        let center = request.Source.Feet |> List.averageBy _.Depth
        let centered = WalkerConstructionTheory.targetOptions request |> List.head
        let selected = centered |> List.find (fun t -> t.Placement.Id=motor.Placement.Id) |> _.Placement
        let bounds = WalkerMotionSpace.partEnvelope selected
        Assert.Equal(center,(bounds.Low.Z+bounds.High.Z)/2.0,6)
        Assert.Equal(placement.Rotation,selected.Rotation)
        Assert.Contains(request.Targets,WalkerConstructionTheory.targetOptions request)
        let routes = WalkerConstructionTheory.motorDriveOptions {request with Targets=centered}
        Assert.NotEmpty routes
        for route in routes do
            Assert.Equal(request.Reference,route.Reference)
            Assert.Equal(request.Program,route.Program)
            Assert.Equal(request.Source,route.Source)
            Assert.Equal(request.Drive.MotorClockScale,route.Drive.MotorClockScale)
            for body,gear in route.Drive.Gears do
                let old = request.Drive.Gears |> List.find (fun (id,_) -> id=body) |> snd
                Assert.Equal(old.Position.X,gear.Position.X)
                Assert.Equal(old.Position.Y,gear.Position.Y)
            for edge in route.Program.Transmissions do
                let meshes = [for a,gearA in route.Drive.Gears do
                                  for b,gearB in route.Drive.Gears do
                                      if a=edge.Input && b=edge.Output
                                         && Catalog.matchesSpurMesh edge.Ratio gearA.Part gearA.Position gearB.Part gearB.Position then
                                          yield gearA,gearB]
                Assert.Single meshes |> ignore
            let root = route.Drive.Gears |> List.find (fun (body,_) -> body=route.Drive.Root) |> snd
            let original = request.Drive.Gears |> List.find (fun (body,_) -> body=request.Drive.Root) |> snd
            Assert.NotEqual(original.Position.Z,root.Position.Z)
            let bearings = WalkerPackaging.bearingPlanes route.Drive.InternalCompoundBearings [root]
            Assert.All(bearings,fun plane ->
                if face>0.0 then Assert.True(plane+10.0<=selected.Position.Z+20.0+1e-6)
                else Assert.True(plane-10.0>=selected.Position.Z-20.0-1e-6))
        let boundedTargets = request.Targets |> List.map (fun t ->
            if t.Placement.Id=motor.Placement.Id then {t with ToleranceLdu=Vec3.create 0 0 80} else t)
        let bounded = {request with Targets=boundedTargets}
        Assert.DoesNotContain(WalkerConstructionTheory.targetOptions bounded,fun targets ->
            (targets |> List.find (fun t -> t.Placement.Id=motor.Placement.Id)).Placement.Position=selected.Position)

    let private forwardFrame (request: WalkerPackagingRequest) =
        let bodies = request.Source.Bodies |> List.map (fun body ->
            if not(request.Program.Bodies |> List.exists (fun b -> b.Id=body.Body && b.Grounded)) then body
            else
                let parts = body.Parts |> List.map (fun p ->
                    if request.Targets |> List.exists (fun t -> t.Placement.Id=p.Id) then p
                    else {p with Position={p.Position with X=if p.Id="bridge-0" then 110.0 else p.Position.X+120.0}})
                {body with Parts=parts})
        {request with Source={request.Source with Bodies=bodies}}

    [<Fact>]
    let ``scaffold alternatives move the backbone without moving walking joints and reconnect using covered pins`` () =
        let program,source = Walker1Seeds.walker 2
        let screened = Realization.screen program 72 |> Result.defaultWith (Realization.json >> failwith)
        let request = WalkerPackaging.rebuildRequest screened source
        let rail = request.Source.Bodies |> List.collect _.Parts |> List.find (fun part -> part.Id.StartsWith "rail-")
        let bodies = request.Source.Bodies |> List.map (fun body ->
            let parts = body.Parts |> List.map (fun part ->
                if part.Id=rail.Id then {part with Id="scaffold-link-0-anchor-beam"} else part)
            {body with Parts=parts})
        let request = {request with Source={request.Source with Bodies=bodies}}
        let source = request.Source
        let alternatives = WalkerScaffold.options request screened |> Result.defaultWith (Realization.json >> failwith)
        Assert.InRange(alternatives.Length,2,WalkerScaffold.maximumAlternatives)
        Assert.Contains(source,alternatives)
        for candidate in alternatives |> List.filter ((<>) source) do
            let ids = (candidate.Bodies |> List.collect _.Parts) @ candidate.Mounts @ (candidate.Connections |> List.map _.Placement) |> List.map _.Id
            Assert.Equal(ids.Length,(Set.ofList ids).Count)
            Assert.Equal<WalkerPivotChoice list>(source.Connections,candidate.Connections)
            Assert.Equal<WalkerFootChoice list>(source.Feet,candidate.Feet)
            let ground = program.Bodies |> List.find _.Grounded |> _.Id
            Assert.Equal<WalkerBodyChoice list>(source.Bodies |> List.filter (fun b -> b.Body<>ground),
                         candidate.Bodies |> List.filter (fun b -> b.Body<>ground))
            let parts = candidate.Bodies |> List.find (fun b -> b.Body=ground) |> _.Parts
            Assert.NotEqual<WalkerPlacement list>((source.Bodies |> List.find (fun b -> b.Body=ground)).Parts,parts)
            let generated = candidate.Mounts |> List.filter (fun p -> p.Id.StartsWith "scaffold-")
            Assert.NotEmpty generated
            let frame = parts |> List.filter (fun p ->
                p.Id.StartsWith "rail-" || p.Id.StartsWith "bridge-"
                || p.Id.Contains "frame-upright-" || p.Id.StartsWith "scaffold-")
            for pin in generated do
                let contacts = frame |> List.collect (fun part ->
                    (Catalog.get part.Part).Ports |> List.choose (fun port ->
                        let point = Vec3.add part.Position (Rotation.apply part.Rotation port.Position)
                        let axis = Rotation.apply part.Rotation port.Axis
                        let along = Vec3.dot (Vec3.subtract point pin.Position) pin.Rotation.X
                        let low,high = Catalog.shaftRange pin.Part
                        if abs(Vec3.dot axis pin.Rotation.X)>0.999
                           && Vec3.length(Vec3.cross (Vec3.subtract point pin.Position) pin.Rotation.X)<1e-6
                           && along>=low && along<=high then Some along else None))
                Assert.Equal(2,contacts.Length)
                for along in contacts do
                    let low,high = Catalog.shaftRange pin.Part
                    Assert.True(along-10.0>=low-1e-6 && along+10.0<=high+1e-6)

    [<Fact>]
    let ``scaffold displacement adjusts measured mass before hardware balancing`` () =
        let request = fixture ()
        let ground = request.Program.Bodies |> List.find _.Grounded |> _.Id
        let rail = request.Source.Bodies |> List.find (fun b -> b.Body=ground) |> _.Parts |> List.find (fun p -> p.Id.StartsWith "rail-")
        let masses = request.Targets |> List.map (fun t ->
            {Part=t.Placement.Id;MassKg=0.05;CenterLdu=t.Placement.Position})
        let model = {TotalMassKg=0.5;CenterOfMassLdu=Vec3.create 100 -50 60
                     SupportCenterLdu=Vec3.create 100 -50 60;SupportHalfSpanLdu=Vec3.create 100 0 60
                     Components=masses;ScaffoldComponents=Some [{Part=rail.Id;MassKg=0.1;CenterLdu=rail.Position}]}
        let bodies = request.Source.Bodies |> List.map (fun b ->
            {b with Parts=b.Parts |> List.map (fun p -> if p.Id=rail.Id then {p with Position=Vec3.add p.Position (Vec3.create 40 -24 20)} else p)})
        let source = {request.Source with Bodies=bodies}
        let corrected = WalkerScaffold.balanceForSource {request with Balance=Some model} source |> Option.get
        Assert.Equal(Vec3.create 108 -54.8 64,corrected.CenterOfMassLdu)

    [<Fact>]
    let ``bounded target assignments retain centered and nominal motor branches despite payload combinations`` () =
        let request = fixture () |> forwardFrame
        let targets = request.Targets |> List.map (fun t ->
            {t with ToleranceLdu=if t.Placement.Id=request.Source.MotorPart then Vec3.create 0 0 240 else Vec3.create 240 240 240})
        let request = {request with Targets=targets}
        let screened = Realization.screen request.Program 72 |> Result.defaultWith (Realization.json >> failwith)
        let options = WalkerConstructionTheory.motionTargetOptions request screened |> Result.defaultWith (Realization.json >> failwith)
        let motor (targets: WalkerComponentTarget list) =
            targets |> List.find (fun t -> t.Placement.Id=request.Source.MotorPart) |> _.Placement
        Assert.InRange(options.Length,2,32)
        Assert.Contains(options,fun targets -> motor targets=motor request.Targets)
        let center = request.Source.Feet |> List.averageBy _.Depth
        Assert.Contains(options,fun targets ->
            let bounds = motor targets |> WalkerMotionSpace.partEnvelope
            abs((bounds.Low.Z+bounds.High.Z)/2.0-center)<1e-6)
        Assert.True((options |> List.map motor |> List.distinct).Length>=5)

    [<Fact>]
    let ``centered motor branch selects a compatible drive before mounting and support search`` () =
        let request = fixture () |> forwardFrame
        let targets = request.Targets |> List.map (fun t ->
            if t.Placement.Id=request.Source.MotorPart then {t with ToleranceLdu=Vec3.create 0 0 240} else t)
        let request = {request with Targets=WalkerConstructionTheory.targetOptions {request with Targets=targets} |> List.head}
        let request = {request with Targets=request.Targets |> List.map (fun t -> {t with ToleranceLdu=Vec3.zero})}
        let initial = WalkerConstructionTheory.start request "centered-motor" 512 64
        let expand = WalkerConstructionTheory.expansion request "centered-motor"
        let mutable proof = initial.Frontier.Head
        for expected in ["motor-layout";"frame";"walking";"targets";"drive"] do
            match expand proof [] with
            | Branch(goal,choices) ->
                Assert.Equal(expected,goal)
                Assert.NotEmpty choices
                proof <- ConstructionSearch.apply proof choices.Head
            | other -> failwith (Realization.json other)
        let selected = proof.Steps |> List.find (fun step -> step.Goal="drive")
        let routes = WalkerConstructionTheory.motorDriveOptions request
        Assert.Contains(routes,fun route -> Export.sha256(Realization.json route.Drive)=selected.Choice)
        match expand proof [] with
        | Branch("motor-mount",choices) -> Assert.NotEmpty choices
        | other -> failwith (Realization.json other)
        Assert.Contains(ConstructionSearch.openGoals proof,fun goal -> goal.Id="oracle" && goal.External)

    [<Fact>]
    let ``centered motor cannot bypass retained frame and walking stock`` () =
        let request = fixture ()
        let targets = request.Targets |> List.map (fun t ->
            if t.Placement.Id=request.Source.MotorPart then {t with ToleranceLdu=Vec3.create 0 0 240} else t)
        let request = {request with Targets=targets}
        let centered = WalkerConstructionTheory.targetOptions request |> List.head
        let motor = centered |> List.find (fun t -> t.Placement.Id=request.Source.MotorPart) |> _.Placement
        let conflicts = ResizeArray<WalkerMotionSpace.Conflict>()
        let inspect produce =
            let inspection: WalkerConstructionInspection = produce ()
            if inspection.Candidate |> List.exists (fun box ->
                box.Part=motor.Id && box.Low=(WalkerMotionSpace.partEnvelope motor).Low) then
                conflicts.AddRange inspection.Conflicts
        let screened = Realization.screen request.Program 72 |> Result.defaultWith (Realization.json >> failwith)
        let options = WalkerConstructionTheory.motionTargetOptionsObserved (Some inspect) request screened
                      |> Result.defaultWith (Realization.json >> failwith)
        Assert.DoesNotContain(options,fun targets ->
            (targets |> List.find (fun t -> t.Placement.Id=motor.Id)).Placement=motor)
        Assert.Contains(conflicts,fun conflict -> conflict.Body="fixed-hardware")

    [<Fact>]
    let ``balance ranks joint layouts before support routing and corrects battery for moved motor and receiver`` () =
        let request = fixture () |> forwardFrame
        let targets = request.Targets |> List.map (fun t ->
            {t with ToleranceLdu=if t.Placement.Id=request.Source.MotorPart then Vec3.create 0 0 240 else Vec3.create 240 240 240})
        let model =
            {TotalMassKg=0.5;CenterOfMassLdu=Vec3.create 80 -50 100
             SupportCenterLdu=Vec3.create 180 -50 60;SupportHalfSpanLdu=Vec3.create 150 0 100
             ScaffoldComponents=None
             Components=request.Targets |> List.map (fun t ->
                let mass = if t.Placement.Id="payload-battery" then 0.2 elif t.Placement.Id=request.Source.MotorPart then 0.06 else 0.025
                let original = request.Source.Bodies |> List.collect _.Parts |> List.find (fun p -> p.Id=t.Placement.Id)
                {Part=t.Placement.Id;MassKg=mass;CenterLdu=original.Position})}
        let request = {request with Targets=targets;Balance=Some model}
        let screened = Realization.screen request.Program 72 |> Result.defaultWith (Realization.json >> failwith)
        let legacy = WalkerConstructionTheory.motionTargetOptions {request with Balance=None} screened
                     |> Result.defaultWith (Realization.json >> failwith)
        let ranked = WalkerConstructionTheory.motionTargetOptions request screened |> Result.defaultWith (Realization.json >> failwith)
        Assert.InRange(ranked.Length,1,32)
        let first = WalkerConstructionTheory.balanceScore request ranked.Head
        Assert.True(first<WalkerConstructionTheory.balanceScore request legacy.Head*0.5)
        Assert.All(ranked,fun targets -> Assert.True(first<=WalkerConstructionTheory.balanceScore request targets+1e-9))
        let positions =
            request.Targets |> List.map (fun t ->
                let delta = if t.Placement.Id=request.Source.MotorPart then Vec3.create 0 0 20
                            elif t.Placement.Id="payload-receiver" then Vec3.create 20 0 0 else Vec3.zero
                {t with Placement={t.Placement with Position=Vec3.add t.Placement.Position delta}})
        let baseline = WalkerConstructionTheory.estimatedCenter request request.Targets |> Option.get
        let center = WalkerConstructionTheory.estimatedCenter request positions |> Option.get
        Assert.Equal(baseline.X+0.025/0.5*20.0,center.X,6)
        Assert.Equal(baseline.Z+0.06/0.5*20.0,center.Z,6)
        Assert.NotEqual<string>(WalkerConstructionTheory.scope request "balance",WalkerConstructionTheory.scope {request with Balance=None} "balance")
        let invalid = {model with TotalMassKg=0.1}
        Assert.Equal("invalid-balance-model",(WalkerPackaging.realize {request with Balance=Some invalid} 1).Head.Gates.Head.Code)
        for invalid in [Unchecked.defaultof<WalkerBalanceModel>
                        {model with Components=Unchecked.defaultof<_>}
                        {model with Components=Unchecked.defaultof<_>::model.Components.Tail}] do
            Assert.Equal("invalid-balance-model",(WalkerPackaging.realize {request with Balance=Some invalid} 1).Head.Gates.Head.Code)

    [<Fact>]
    let ``shifted motor mount bonds refer to the selected placement rather than nominal targets`` () =
        let request = fixture ()
        let rootGear = request.Drive.Gears |> List.find (fun (body,_) -> body=request.Drive.Root) |> snd
        let targets = request.Targets |> List.map (fun t ->
            if t.Placement.Id=request.Source.MotorPart then
                let position = {rootGear.Position with Z=rootGear.Position.Z-20.0}
                let rotation = {X=Vec3.create -1 0 0;Y=Vec3.create 0 1 0;Z=Vec3.create 0 0 -1}
                let placed = {t.Placement with Position=position;Rotation=rotation}
                {t with Placement=placed;ToleranceLdu=Vec3.create 0 0 80}
            else t)
        let request = {request with Targets=targets}
        let state,witness =
            WalkerConstructionTheory.start request "shifted-mount-test" 4096 64
            |> WalkerConstructionTheory.advance request "shifted-mount-test"
        Assert.True(witness.IsSome,Realization.json state.Rejections)
        let construction = witness.Value
        let placed = construction.Realized.Assembly.Parts |> List.find (fun p -> p.Id=request.Source.MotorPart)
        let nominal = targets |> List.find (fun t -> t.Placement.Id=request.Source.MotorPart) |> _.Placement
        Assert.NotEqual(nominal.Position.Z,placed.Position.Z)
        let bonds = state.Pending.Value.Steps |> List.collect _.Bonds |> List.filter (fun b -> b.PartB=placed.Id)
        Assert.Equal(2,bonds.Length)
        for bond in bonds do
            Assert.Contains((Catalog.get placed.Part).Ports,fun port ->
                Vec3.distance bond.PositionB (Vec3.add placed.Position (Rotation.apply placed.Rotation port.Position))<1e-6)

    [<Fact>]
    let ``nominal shaft contact budget counts structural parts and future bearings`` () =
        let gear = (fixture ()).Drive.Gears.Head |> snd
        let parts = [100.0;120.0;160.0] |> List.mapi (fun index depth ->
            {gear with Id=string index;Position=Vec3.create 0 0 depth;Rotation=Rotation.identity})
        let axle = {gear with
                        Part="3705.dat"
                        Position=Vec3.create 0 0 130
                        Rotation={X=Vec3.create 0 0 1;Y=Vec3.create 1 0 0;Z=Vec3.create 0 1 0}}
        Assert.Equal(1,WalkerPackaging.contactCapacityDeficit axle parts 2)
        Assert.Equal(0,WalkerPackaging.contactCapacityDeficit {axle with Part="44294.dat"} parts 2)
        let pins = parts |> List.map (fun p -> {p with Position={p.Position with Z=130.0}})
        Assert.Equal(1,WalkerPackaging.contactCapacityDeficit {axle with Part="4459.dat"} pins 0)
        Assert.Equal(0,WalkerPackaging.contactCapacityDeficit {axle with Part="6558.dat"} pins 0)
        let nearby = pins |> List.map (fun p -> {p with Position={p.Position with X=2.5}})
        Assert.Equal(1,WalkerPackaging.contactCapacityDeficit {axle with Part="4459.dat"} nearby 0)
        let outside = pins |> List.map (fun p -> {p with Position={p.Position with X=3.1}})
        Assert.Equal(0,WalkerPackaging.contactCapacityDeficit {axle with Part="4459.dat"} outside 0)

    [<Fact>]
    let ``internal compound bearings allow a short shaft without relaxing gear wall coverage`` () =
        let gear = (fixture ()).Drive.Gears.Head |> snd
        let gears = [{gear with Position={gear.Position with Z=100.0}};
                     {gear with Id=gear.Id+"-compound";Position={gear.Position with Z=160.0}}]
        Assert.Equal<float list>([80.0;120.0;140.0;180.0],WalkerPackaging.bearingPlanes false gears)
        Assert.Equal<float list>([120.0;140.0],WalkerPackaging.bearingPlanes true gears)
        let low,high = Catalog.shaftRange "3705.dat"
        Assert.Equal(90.0,130.0+low)
        Assert.Equal(170.0,130.0+high)
        for center in [100.0;120.0;140.0;160.0] do
            Assert.True(130.0+low<=center-10.0 && 130.0+high>=center+10.0)
        Assert.True(130.0+high+8.0<195.0)
        Assert.Throws<System.ArgumentException>(fun () ->
            WalkerPackaging.bearingPlanes true [{gear with Position={gear.Position with Z=100.0}};
                                                {gear with Position={gear.Position with Z=140.0}}] |> ignore) |> ignore

    [<Fact>]
    let ``compound stages can move to the central shaft without moving loaded outputs or motor plane`` () =
        let request = fixture ()
        let gear = request.Drive.Gears.Head |> snd
        let bodies = ["motor-root";"shaft-2";"shaft-3";"shaft-4";"shaft-5";"shaft-6"]
        let gears = bodies |> List.mapi (fun index body ->
            let position = if body="motor-root" then Vec3.create 180 -60 160 else Vec3.create (float index*60.0) 0 160
            body,{gear with Id="gear-"+body;Position=position})
        let loads = ["shaft-2";"shaft-6"] |> List.map (fun body ->
            let placement = gears |> List.find (fun (id,_) -> id=body) |> snd
            {request.Source.Bodies.Head with Body=body;Parts=[{placement with Id="load-"+body;Position={placement.Position with Z=60.0}}]})
        let edges = ["motor-root","shaft-4";"shaft-4","shaft-3";"shaft-3","shaft-2";"shaft-4","shaft-5";"shaft-5","shaft-6"]
                    |> List.map (fun (input,output) ->
                        {request.Program.Transmissions.Head with Input=input;Output=output})
        let request = {request with Source={request.Source with Bodies=loads};ObsoletePartIds=[]
                                    Program={request.Program with Transmissions=edges}
                                    Drive={request.Drive with Root="motor-root";Gears=gears}}
        let pin = request.Source.Connections.Head
        let pivot = {pin.Placement with
                        Part="3673.dat"
                        Position=Vec3.create 120 0 170
                        Rotation={X=Vec3.create 0 0 1;Y=Vec3.create 1 0 0;Z=Vec3.create 0 1 0}}
        let request = {request with Source={request.Source with Connections=[{pin with Placement=pivot}]}}
        let options = WalkerConstructionTheory.driveOptions request
        let misplaced = gears |> List.map (fun (body,gear) ->
            if body="motor-root" then body,{gear with Position=Vec3.create 0 0 160} else body,gear)
        Assert.Empty(WalkerConstructionTheory.driveOptions {request with Drive={request.Drive with Gears=misplaced}})
        let centered = options |> List.find (fun route ->
            route.Drive.InternalCompoundBearings &&
            (route.Drive.Gears |> List.filter (fun (body,_) -> body="shaft-4") |> List.length)=2)
        let planes body = centered.Drive.Gears |> List.filter (fun (id,_) -> id=body) |> List.map (snd >> _.Position.Z) |> List.sort
        for body in ["shaft-2";"shaft-3";"shaft-5";"shaft-6"] do Assert.Equal<float list>([100.0],planes body)
        Assert.Equal<float list>([100.0;160.0],planes "shaft-4")
        Assert.Equal<float list>([160.0],planes "motor-root")
        Assert.Equal(request.Program,centered.Program)
        Assert.Equal(centered.Drive,options.Head.Drive)
        let low,high = Catalog.shaftRange "3705.dat"
        Assert.True(100.0+low<=70.0 && 100.0+high>=130.0)
        Assert.True(100.0+high+8.0<150.0)

    [<Fact>]
    let ``joint engagement feedback reaches the existing smooth pin repair grammar`` () =
        let request = fixture ()
        let screened = Realization.screen request.Reference 72 |> Result.defaultWith (Realization.json >> failwith)
        let pivot = request.Source.Connections |> List.find (fun c -> (Catalog.get c.Placement.Part).Connector=Some FrictionlessPin)
        let feedback = {Kind="joint-port-engagement";Parts=[pivot.Placement.Id];Bodies=[]}
        let choices = WalkerConstruction.repairs screened request.Source feedback 8
        let legacy = WalkerConstruction.repairs screened request.Source {feedback with Kind="rigid-body-partition"} 8
        Assert.Equal(Realization.json legacy,Realization.json choices)

    [<Fact>]
    let ``motor mounts use housing side sockets and perpendicular block witnesses`` () =
        let ports = Catalog.mountPorts "58121.dat"
        Assert.Equal(2,ports.Length)
        Assert.True(ports |> List.forall (fun p -> abs p.Axis.X=1.0 && p.Position.Z=10.0))
        let request = fixture ()
        let state,witness = WalkerConstructionTheory.start request "motor-mount-oracle" 512 64
                            |> WalkerConstructionTheory.advance request "motor-mount-oracle"
        Assert.True(witness.IsSome,Realization.json state.Rejections)
        let parts = witness.Value.Realized.Genome.Bodies |> List.collect _.Parts
        Assert.Equal(2,parts |> List.filter (fun p -> p.Part="6536.dat") |> List.length)
        let bonds = state.Pending.Value.Steps |> List.collect _.Bonds
        let motorBonds = bonds |> List.filter (fun b -> b.PartB=request.Source.MotorPart)
        Assert.Equal(2,motorBonds.Length)
        Assert.True(motorBonds |> List.forall (fun b ->
            b.PartA.EndsWith("-corner") && abs b.AxisB.X>0.999 && b.Connector.EndsWith("-side-pin")))
        let proof = state.Pending.Value
        let kinds = proof.Goals |> List.map (fun goal -> goal.Id,goal.Kind) |> Map.ofList
        let axles = proof.Steps |> List.filter (fun step -> kinds[step.Goal]="axle-stock") |> List.map _.Goal
        let mutable prior = []
        for step in proof.Steps do
            if kinds[step.Goal]="mount-and-bearing" then
                Assert.True(axles @ prior |> List.forall (fun id -> List.contains id step.Dependencies))
                prior <- prior @ [step.Goal]

    [<Fact>]
    let ``nine L stock covers the loaded output walls without crossing the neighboring link planes`` () =
        Assert.Equal(Some Axle,(Catalog.get "60485.dat").Connector)
        let low,high = Catalog.shaftRange "60485.dat"
        Assert.Equal(-90.0,low)
        Assert.Equal(90.0,high)
        let center = 60.0
        let minimum,maximum = center+low,center+high
        Assert.Equal(-30.0,minimum)
        Assert.Equal(150.0,maximum)
        for socket in [-20.0;140.0] do
            Assert.True(minimum<=socket-10.0 && maximum>=socket+10.0)
        for bearing in [80.0;120.0] do
            Assert.True(minimum<=bearing-10.0 && maximum>=bearing+10.0)
        Assert.Equal(10.0,minimum-(-40.0))
        Assert.Equal(10.0,160.0-maximum)
        let shortLow,shortHigh = Catalog.shaftRange "3707.dat"
        Assert.True(shortHigh-shortLow<maximum-minimum)

    [<Fact>]
    let ``shared theory opens actual stock and support decisions before external admission`` () =
        let request = fixture ()
        let initial = WalkerConstructionTheory.start request "test-oracle-and-library" 512 64
        let state,witness = WalkerConstructionTheory.advance request "test-oracle-and-library" initial
        Assert.True(witness.IsSome,$"{state.Status}; {state.Visited} states; {Realization.json(state.Rejections |> List.truncate 5)}")
        let proof = state.Pending.Value
        let motorMount = proof.Steps |> List.find (fun s -> s.Goal="motor-mount")
        Assert.Equal("select-stock-motor-mount-motif",motorMount.Rule)
        Assert.Empty(motorMount.Parts)
        Assert.True(proof.Steps |> List.exists (fun s -> s.Rule="stock-axle-wall-coverage"))
        Assert.True(proof.Steps |> List.exists (fun s -> s.Rule="two-beam-covered-pin-bracket"))
        Assert.Equal("oracle",(ConstructionSearch.openGoals proof).Head.Id)
        Assert.Contains(ConstructionSearch.openGoals proof,fun g -> g.Id="library-connectivity")
        let walking = proof.Steps |> List.find (fun s -> s.Goal="walking")
        let drive = proof.Steps |> List.find (fun s -> s.Goal="drive")
        let preserved = proof.Goals |> List.filter (fun g -> g.Kind="exact-joint-preservation")
        let expectedPivots = witness.Value.Realized.Genome.Connections |> List.filter (fun c -> List.contains c.Placement.Id walking.Parts)
        Assert.Equal<Set<string>>(expectedPivots |> List.map (fun c -> c.Placement.Id) |> Set.ofList,
                                  preserved |> List.map _.Subject |> Set.ofList)
        Assert.NotEmpty(preserved)
        for goal in preserved do
            Assert.Contains(goal.Id,drive.Introduced)
            Assert.True(goal.External)
            for producer in proof.Steps |> List.filter (fun s -> not s.Parts.IsEmpty) do
                Assert.Contains(producer.Goal,goal.Requires)
        Assert.True(proof.Steps |> List.exists (fun s -> not s.Bonds.IsEmpty))
        let chassis = witness.Value.Realized.Assembly.Chassis
        let frame = proof.Steps |> List.find (fun s -> s.Goal="frame") |> _.Parts |> Set.ofList
        let placements =
            (witness.Value.Realized.Genome.Bodies |> List.collect _.Parts) @ witness.Value.Realized.Genome.Mounts
            |> List.map (fun part -> part.Id,part) |> Map.ofList
        let retained = witness.Value.RetainedFramePartIds |> List.map (fun id -> placements[id])
        let targets = request.Targets |> List.map (fun target -> placements[target.Placement.Id])
        let supports = proof.Steps |> List.filter (fun step ->
            proof.Goals |> List.exists (fun goal -> goal.Id=step.Goal && goal.Kind="mount-and-bearing"))
        for step in supports do
            let parts = step.Parts |> List.map (fun id -> placements[id])
            Assert.True((WalkerConstructionTheory.supportRouteGate step.Goal parts retained targets).IsNone)
        let selected = supports.Head
        let parts = selected.Parts |> List.map (fun id -> placements[id])
        let broken = parts |> List.map (fun part ->
            if part.Id=selected.Goal+"-anchor-pin" then {part with Position=Vec3.add part.Position (Vec3.create 2000 0 0)}
            else part)
        let rejected = WalkerConstructionTheory.supportRouteGate selected.Goal broken retained targets
        Assert.True(rejected.IsSome)
        Assert.Equal("unproved-rigid-pin-bond",rejected.Value.Code)
        let newStructural =
            witness.Value.Realized.Genome.Bodies |> List.find (fun b -> b.Body=chassis) |> _.Parts
            |> List.map _.Id |> Set.ofList |> fun parts -> Set.difference parts frame
        Assert.Equal<Set<string>>(newStructural,
            proof.Goals |> List.filter (fun g -> g.Kind="rigid-path-to-frame") |> List.map _.Subject |> Set.ofList)
        Assert.Throws<System.ArgumentException>(fun () -> ConstructionSearch.accept "ordinary-load-only-report" state |> ignore) |> ignore
        Assert.Throws<System.ArgumentException>(fun () ->
            ConstructionSearch.acceptVerified "report" ["oracle";"library-connectivity"] state |> ignore) |> ignore
        let _,accepted = ConstructionSearch.acceptVerified "report" (ConstructionSearch.openGoals proof |> List.map _.Id) state
        Assert.Empty(ConstructionSearch.openGoals accepted)
        Assert.Equal<Set<string>>(witness.Value.Realized.Assembly.Parts |> List.map _.Id |> Set.ofList,
                                  proof.Steps |> List.collect _.Parts |> Set.ofList)
        Assert.Throws<System.ArgumentException>(fun () ->
            WalkerConstructionTheory.advance request "different-library" state |> ignore) |> ignore

    [<Fact>]
    let ``packaging validates bounded search budget`` () =
        let request = fixture ()
        Assert.Throws<System.ArgumentException>(fun () -> WalkerPackaging.realize request 0 |> ignore) |> ignore
        Assert.Throws<System.ArgumentException>(fun () -> WalkerPackaging.realize request 3 |> ignore) |> ignore

    [<Fact>]
    let ``three L friction pins are genuine alternatives when a two L pin fits`` () =
        let request = fixture ()
        let oracle = "pin-stock-oracle"
        let initial = WalkerConstructionTheory.start request oracle 512 64
        let state,original = WalkerConstructionTheory.advance request oracle initial
        Assert.True(original.IsSome)
        let proof = state.Pending.Value
        let expand = WalkerConstructionTheory.expansion request oracle
        let originalParts =
            (original.Value.Realized.Genome.Bodies |> List.collect _.Parts)
            @ original.Value.Realized.Genome.Mounts
            |> List.map (fun p -> p.Id,p) |> Map.ofList
        let longer =
            proof.Steps |> List.indexed |> List.filter (fun (_,s) -> s.Rule="two-beam-covered-pin-bracket")
            |> List.tryPick (fun (index,selected) ->
                let prefix =
                    proof.Steps |> List.take index
                    |> List.fold (fun prefix step ->
                        let introduced = step.Introduced |> List.map (fun id -> proof.Goals |> List.find (fun goal -> goal.Id=id))
                        ConstructionSearch.apply prefix {Step=step;Introduces=introduced}) initial.Frontier.Head
                let choices =
                    match expand prefix [] with
                    | Branch(_,choices) -> choices
                    | _ -> failwith "Expected an open bracket production."
                let parts = selected.Parts |> List.map (fun id -> originalParts[id])
                parts |> List.filter (fun p -> p.Part="4459.dat")
                |> List.tryPick (fun pin ->
                    let upgraded = parts |> List.map (fun p -> if p.Id=pin.Id then {p with Part="6558.dat"} else p)
                    let key = Export.sha256(Realization.json upgraded)
                    choices |> List.tryFind (fun c -> c.Step.Choice=key)
                    |> Option.map (fun choice -> prefix,choice)))
        Assert.True(longer.IsSome,"Expected a same-geometry 2L-to-3L pin stock alternative.")
        let prefix,choice = longer.Value
        let frontier = ConstructionSearch.apply prefix choice
        let next = {initial with Frontier=[frontier]}
        let _,alternative = ConstructionSearch.next expand next
        Assert.True(alternative.IsSome)
        Assert.True(alternative.Value.Realized.Genome.Mounts |> List.exists (fun pin ->
            match Map.tryFind pin.Id originalParts with
            | Some old -> old.Part="4459.dat" && pin.Part="6558.dat" && old.Position=pin.Position && old.Rotation=pin.Rotation
            | None -> false))

    [<Fact>]
    let ``missing targets and duplicate drive stock are explicit gates`` () =
        let request = fixture ()
        let missing = WalkerPackaging.realize {request with Targets=request.Targets.Tail} 1
        Assert.Equal("invalid-component-targets",missing.Head.Gates.Head.Code)
        Assert.True(missing.Head.Construction.IsNone)
        let duplicate = WalkerPackaging.realize {request with Drive={request.Drive with Gears=request.Drive.Gears.Head::request.Drive.Gears}} 1
        Assert.Equal("incomplete-drive-stock",duplicate.Head.Gates.Head.Code)

    [<Fact>]
    let ``invalid tolerances and obsolete references are explicit gates`` () =
        let request = fixture ()
        let targets = request.Targets |> List.map (fun t -> {t with ToleranceLdu=Vec3.create -1 0 0})
        Assert.Equal("invalid-component-targets",(WalkerPackaging.realize {request with Targets=targets} 1).Head.Gates.Head.Code)
        Assert.Equal("invalid-obsolete-parts",(WalkerPackaging.realize {request with ObsoletePartIds=["unknown"]} 1).Head.Gates.Head.Code)

    [<Fact>]
    let ``changing feet or motor clock is rejected before building`` () =
        let request = fixture ()
        let badFeet = {request with Program={request.Program with Feet=request.Program.Feet.Tail}}
        Assert.Equal("changed-walking-requirements",(WalkerPackaging.realize badFeet 1).Head.Gates.Head.Code)
        let badClock = {request with Drive={request.Drive with MotorClockScale=2.0}}
        Assert.Equal("changed-walking-requirements",(WalkerPackaging.realize badClock 1).Head.Gates.Head.Code)

    [<Fact>]
    let ``realization emits physical supports and preserves target and foot definitions`` () =
        let request = fixture ()
        let attempts = WalkerPackaging.realize request 2
        let results = attempts |> List.choose _.Construction
        Assert.True(not results.IsEmpty,Realization.json(attempts |> List.map _.Gates))
        for result in results do
            Assert.True(result.AddedPartIds.Length>0)
            Assert.True(result.RetainedFramePartIds.Length>0)
            Assert.True(Grammar.validateAssembly result.Realized.Assembly |> List.isEmpty)
            Assert.Equal<WalkerFootChoice list>(request.Source.Feet,result.Realized.Genome.Feet)
            for target in request.Targets do
                let actual = result.Realized.Genome.Bodies |> List.collect _.Parts |> List.find (fun p -> p.Id=target.Placement.Id)
                Assert.Equal(target.Placement,actual)
            for part in result.Realized.Assembly.Parts do
                if List.contains part.Id request.ObsoletePartIds then
                    Assert.Equal(Some Axle,(Catalog.get part.Part).Connector)
                    Assert.Contains(part.Id,result.AddedPartIds)
            for axle in result.Realized.Genome.Connections |> List.filter (fun c -> (Catalog.get c.Placement.Part).Connector=Some Axle) do
                let low,high = Catalog.shaftRange axle.Placement.Part
                let body = result.Screened.Program.Hinges |> List.find (fun h -> h.Id=axle.Hinges.Head)
                let driven = if body.A.Body=result.Realized.Assembly.Chassis then body.B.Body else body.A.Body
                let gear = result.Realized.Genome.Bodies |> List.find (fun b -> b.Body=driven) |> _.Parts
                           |> List.find (fun p -> (Catalog.gearTeeth p.Part).IsSome)
                Assert.True(axle.Placement.Position.Z+low<=gear.Position.Z-30.0+1e-6)
                Assert.True(axle.Placement.Position.Z+high>=gear.Position.Z+30.0-1e-6)
                if driven=request.Drive.Root then
                    let motor = request.Targets |> List.find (fun t -> t.Placement.Id=request.Source.MotorPart) |> _.Placement
                    if motor.Rotation.Z.Z>0.0 then Assert.True(axle.Placement.Position.Z+high<=motor.Position.Z+20.0+1e-6)
                    else Assert.True(axle.Placement.Position.Z+low>=motor.Position.Z-20.0-1e-6)
