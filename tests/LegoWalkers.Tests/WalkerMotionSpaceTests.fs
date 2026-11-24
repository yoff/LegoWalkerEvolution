module LegoWalkers.WalkerMotionSpaceTests

open System
open LegoWalkers
open Xunit

let private unwrap result = Result.defaultWith (Realization.json >> failwith) result

let private fixture () =
    let program,genome = WalkerSeeds.fourBar 2
    let s = Realization.screen program 72 |> unwrap
    let root = s.Frames.Head.Poses[program.Motor.Body].Origin
    let part = {Id="swept-link";Part="32523.dat";Color=7;Rotation=Rotation.identity
                Position=Vec3.add root (Vec3.create 100 0 500)}
    let g = {genome with Bodies=[{Body=program.Motor.Body;Parts=[part]}];Connections=[];Mounts=[]}
    let target = {part with Id="new-support";Part="6536.dat";Position=Vec3.add root (Vec3.create 0 100 500)}
    s,g,target

[<Fact>]
let ``reservation blocks hardware that is clear initially but crossed later in the gait`` () =
    let s,g,target = fixture ()
    let space = WalkerMotionSpace.reserve s g |> unwrap
    let conflicts = WalkerMotionSpace.conflicts space "chassis" [target]
    Assert.NotEmpty conflicts
    Assert.All(conflicts,fun c -> Assert.True(c.InputAngleRadians>0.0))
    Assert.False(WalkerMotionSpace.fits space "chassis" [target])
    let moved = {target with Position=Vec3.add target.Position (Vec3.create 300 0 0)}
    Assert.True(WalkerMotionSpace.fits space "chassis" [moved])
    Assert.Equal(0,(WalkerMotionSpace.describe space).dynamicTrials)

[<Fact>]
let ``new bearing axes remain obstacles outside the original swept bounds`` () =
    let s,g,target = fixture ()
    let space = WalkerMotionSpace.reserve s g |> unwrap
    let moved = {target with Position=Vec3.add target.Position (Vec3.create 300 0 0)}
    Assert.True(WalkerMotionSpace.fits space "chassis" [moved])
    let axis = {moved with Id="new-bearing-axis";Part="3705.dat"}
    let extended = WalkerMotionSpace.addBearingAxis space s.Program.Motor.Body axis
    Assert.False(WalkerMotionSpace.fits extended "chassis" [moved])
    let conflict = WalkerMotionSpace.conflicts extended "chassis" [moved] |> List.exactlyOne
    Assert.Equal(axis.Id,conflict.Obstacle)

[<Fact>]
let ``reservation covers five motor turns of a reduced walking gait`` () =
    let s,g,_ = fixture ()
    let edge = s.Program.Transmissions |> List.find (fun t -> t.Input=s.Program.Motor.Body)
    let ratio = edge.Ratio/5.0
    let phase = s.Frames.Head.Poses[edge.Output].Angle-ratio*s.Frames.Head.Poses[edge.Input].Angle
    let transmissions = s.Program.Transmissions |> List.map (fun t ->
        if t=edge then {t with Ratio=ratio;Phase=phase} else t)
    let program = {s.Program with Transmissions=transmissions}
    let child = Realization.screen program 72 |> unwrap
    let report = WalkerMotionSpace.reserve child {g with MechanismId=child.MechanismId} |> unwrap |> WalkerMotionSpace.describe
    Assert.Equal(5.0,report.motorTurnsCovered,8)
    Assert.Equal(361,report.sampledPoses)

[<Fact>]
let ``motor motifs use declared front or rear side sockets with covered stock pins`` () =
    let s,_,template = fixture ()
    for sign in [1.0;-1.0] do
        let motor = {template with Id="test-motor";Part="58121.dat";Position=s.Frames.Head.Poses[s.Program.Motor.Body].Origin
                                   Rotation={X=Vec3.create sign 0 0;Y=Vec3.create 0 1 0;Z=Vec3.create 0 0 sign}}
        let options = WalkerPackaging.motorMountOptions motor
        Assert.Equal<string list>(["front-forward";"front-rearward";"rear-saddle"],options |> List.map fst)
        for name,parts in options do
            Assert.Equal(6,parts.Length)
            Assert.True(WalkerMotionSpace.fitsFixed parts [motor])
            for corner in parts |> List.filter (fun p -> p.Part="6536.dat") do
                let prefix = corner.Id.Replace("-corner","")
                let sidePin = parts |> List.find (fun p -> p.Id=prefix+"-side-pin")
                let mountPin = parts |> List.find (fun p -> p.Id=prefix+"-mount-pin")
                let motorSocket = Vec3.subtract corner.Position (Vec3.scale 20.0 sidePin.Rotation.X)
                let depth = if name="rear-saddle" then 110.0 else 10.0
                Assert.Contains((Catalog.get motor.Part).Ports,fun port ->
                    port.Kind=RoundHole && abs(port.Position.Z-depth)<1e-6
                    && Vec3.distance motorSocket (Vec3.add motor.Position (Rotation.apply motor.Rotation port.Position))<1e-6)
                let mount = Vec3.add corner.Position (Rotation.apply corner.Rotation (Vec3.create 0 20 0))
                for pin,ends in [sidePin,[corner.Position;motorSocket]
                                 mountPin,[mount;Vec3.subtract (Vec3.scale 2.0 mountPin.Position) mount]] do
                    let low,high = Catalog.shaftRange pin.Part
                    for point in ends do
                        let delta = Vec3.subtract point pin.Position
                        let along = Vec3.dot delta pin.Rotation.X
                        Assert.True(Vec3.length(Vec3.cross delta pin.Rotation.X)<1e-6)
                        Assert.True(along-10.0>=low-1e-6 && along+10.0<=high+1e-6)

[<Fact>]
let ``fixed motor housings block support beams even on a shared rigid chassis`` () =
    let _,_,template = fixture ()
    let motor = {template with Id="fixed-motor";Part="58121.dat";Position=Vec3.zero}
    let beam = {template with Id="crossing-beam";Part="32523.dat";Position=Vec3.create 0 0 60}
    Assert.False(WalkerMotionSpace.fitsFixed [beam] [motor])
    let conflict = WalkerMotionSpace.fixedConflicts [beam] [motor] |> List.exactlyOne
    Assert.Equal("fixed-hardware",conflict.Body)
    Assert.Equal(motor.Id,conflict.Obstacle)
    Assert.True(WalkerMotionSpace.fitsFixed [{beam with Position=Vec3.create 0 200 60}] [motor])
    Assert.True(WalkerMotionSpace.fitsFixed [motor] [motor])

[<Fact>]
let ``rearward motor motifs avoid the leg layer crossed by forward mounting pins`` () =
    let s,g,template = fixture ()
    for sign in [1.0;-1.0] do
        let motor = {template with Id="test-motor";Part="58121.dat"
                                   Position=Vec3.add s.Frames.Head.Poses[s.Program.Motor.Body].Origin (Vec3.create 0 0 500)
                                   Rotation={X=Vec3.create sign 0 0;Y=Vec3.create 0 1 0;Z=Vec3.create 0 0 sign}}
        let options = WalkerPackaging.motorMountOptions motor
        let forward = options.Head |> snd
        let pin = forward |> List.find (fun p -> p.Id.EndsWith "-mount-pin")
        let blocker = {template with Id="leg-layer";Position={pin.Position with Z=pin.Position.Z-10.0*sign}}
        let source = {g with Bodies=[{Body=s.Program.Motor.Body;Parts=[blocker]}];Connections=[];Mounts=[]}
        let space = WalkerMotionSpace.reserve s source |> unwrap
        Assert.False(WalkerMotionSpace.fits space "chassis" forward)
        for _,parts in options.Tail do
            Assert.True(WalkerMotionSpace.fits space "chassis" parts)

[<Fact>]
let ``space-aware routing continues to longer topologies when shorter routes are obstructed`` () =
    let routes = WalkerPackaging.motionSupportRoutes (fun (parts,_) -> parts.Length>=4)
                    1 "motion-aware" [Vec3.create 0 0 100] [] (Vec3.create 120 0 210) 210.0
    Assert.NotEmpty routes
    Assert.All(routes,fun (parts,_) -> Assert.True(parts.Length>=4))
    Assert.InRange(routes.Length,1,8)

[<Fact>]
let ``unsafe bridges are pruned while the opposite routing direction remains available`` () =
    let mutable prunedBridge = false
    let accepts (parts: WalkerPlacement list,pins: WalkerPlacement list) =
        let clear = parts @ pins |> List.forall (fun p -> p.Position.X<=120.0+1e-6)
        if parts.Length=1 && not clear then prunedBridge <- true
        clear
    let anchors,socket = [Vec3.create 0 0 100],Vec3.create 120 0 290
    let routes = WalkerPackaging.motionSupportRoutesWithPruning accepts 1 "pruned-prefix" anchors [] socket socket.Z
    Assert.True(prunedBridge)
    Assert.NotEmpty routes
    Assert.All(routes,fun route -> Assert.True(accepts route))
    Assert.All(routes,fun (parts,_) -> Assert.Equal(5,parts.Length))
    Assert.InRange(routes.Length,1,8)

[<Fact>]
let ``builder requests retain motion control feet and explicit bounded hardware movement`` () =
    let program,g = WalkerSeeds.fourBar 2
    let s = Realization.screen program 72 |> unwrap
    let request = WalkerPackaging.rebuildRequest s g
    Assert.Equal(s.Program,request.Program)
    Assert.Equal(g,request.Source)
    Assert.Equal(1.0,request.Drive.MotorClockScale)
    Assert.Equal(3,request.Targets.Length)
    let options = WalkerConstructionTheory.motionTargetOptions request s |> unwrap
    for targets in options do
        for target in targets do
            let original = request.Targets |> List.find (fun t -> t.Placement.Id=target.Placement.Id)
            let delta = Vec3.subtract target.Placement.Position original.Placement.Position
            Assert.True(abs delta.X<=original.ToleranceLdu.X && abs delta.Y<=original.ToleranceLdu.Y && abs delta.Z<=original.ToleranceLdu.Z)
            Assert.Equal(original.Placement.Rotation,target.Placement.Rotation)

let private groundFixture () =
    let program,genome = Walker1Seeds.walker 2
    let screened = Realization.screen program 72 |> unwrap
    let feet,_ = WalkerFootPlates.prepare screened genome |> unwrap
    let space = WalkerMotionSpace.reserveWithGround (Some WalkerMotionSpace.defaultGroundClearance) screened feet |> unwrap
    let plan = (WalkerMotionSpace.describe space).groundPlan.Value
    let minimumY = plan.Phases |> List.map _.GroundYLdu |> List.min
    let support = {Id="ground-test-support";Part="3460.dat";Color=7;Rotation=Rotation.identity
                   Position=Vec3.create 10000 (minimumY-plan.Requirements.MinimumNonFootClearanceLdu-8.0) 500}
    screened,feet,space,plan,support

[<Fact>]
let ``ground reservation enforces the specified clearance at every gait phase`` () =
    let _,_,space,plan,support = groundFixture ()
    Assert.True(WalkerMotionSpace.fits space "chassis" [support])
    let lower = {support with Position={support.Position with Y=support.Position.Y+0.01}}
    let failures = WalkerMotionSpace.conflicts space "chassis" [lower]
    Assert.NotEmpty failures
    Assert.All(failures,fun failure -> Assert.Equal("planned-ground-clearance",failure.Obstacle))
    Assert.All(plan.Phases,fun phase -> Assert.NotEmpty phase.StanceFeet)
    Assert.All(plan.Phases,fun phase ->
        Assert.Equal(2,phase.SwingClearanceLdu.Length)
        Assert.All(phase.SwingClearanceLdu,fun (_,lift) -> Assert.True(lift>=0.0))
        Assert.Contains(phase.SwingClearanceLdu,fun (_,lift) -> lift<1e-6))

[<Fact>]
let ``attitude allowance accounts for support overhang about the stance region`` () =
    let s,g,space,_,support = groundFixture ()
    Assert.True(WalkerMotionSpace.fits space "chassis" [support])
    let requirements = {WalkerMotionSpace.defaultGroundClearance with MaximumPitchRadians=Math.PI/18.0}
    let tilted = WalkerMotionSpace.reserveWithGround (Some requirements) s g |> unwrap
    Assert.False(WalkerMotionSpace.fits tilted "chassis" [support])

[<Fact>]
let ``whole assembly ground audit rejects low nonfoot parts without foot linked exemptions`` () =
    let s,g,_,plan,support = groundFixture ()
    let lower = {support with Position={support.Position with Y=support.Position.Y+1.0}}
    let bodies = g.Bodies |> List.map (fun body ->
        if body.Body="chassis" then {body with Parts=body.Parts @ [lower]} else body)
    let candidate = {g with Bodies=bodies}
    let failures = WalkerMotionSpace.auditGround plan s candidate
    Assert.Contains(failures,fun failure -> failure.Part=lower.Id)
    let foreign = {plan with MechanismId="foreign"}
    Assert.Throws<ArgumentException>(fun () -> WalkerMotionSpace.auditGround foreign s candidate |> ignore) |> ignore
    let changedDatum = {plan with Phases=plan.Phases |> List.map (fun phase -> {phase with GroundYLdu=phase.GroundYLdu+1.0})}
    Assert.Throws<ArgumentException>(fun () -> WalkerMotionSpace.auditGround changedDatum s candidate |> ignore) |> ignore
    let invalidRequirements = {plan with Requirements={plan.Requirements with MinimumNonFootClearanceLdu=Double.NaN}}
    Assert.Throws<ArgumentException>(fun () -> WalkerMotionSpace.auditGround invalidRequirements s candidate |> ignore) |> ignore

[<Theory>]
[<InlineData(-1.0)>]
[<InlineData(Double.NaN)>]
[<InlineData(Double.PositiveInfinity)>]
let ``invalid ground requirements are rejected before routing`` clearance =
    let s,g,_,_,_ = groundFixture ()
    let requirements = {WalkerMotionSpace.defaultGroundClearance with MinimumNonFootClearanceLdu=clearance}
    Assert.Throws<ArgumentException>(fun () -> WalkerMotionSpace.groundPlan requirements s g |> ignore) |> ignore

[<Fact>]
let ``default rebuild requests upgrade supported feet while retain policy preserves the source`` () =
    let program,g = Walker1Seeds.walker 2
    let s = Realization.screen program 72 |> unwrap
    let upgraded = WalkerPackaging.rebuildRequest s g
    Assert.NotEqual<WalkerGenome>(g,upgraded.Source)
    Assert.Equal(Some WalkerMotionSpace.defaultGroundClearance,upgraded.GroundClearance)
    Assert.All(upgraded.FootConstructionGates,fun gate -> Assert.Equal("pass",gate.Outcome))
    let retained = WalkerPackaging.rebuildRequestWithFeet false s g
    Assert.Equal<WalkerGenome>(g,retained.Source)
