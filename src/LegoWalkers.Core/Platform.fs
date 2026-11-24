namespace LegoWalkers

open System

type PlatformAssembly = {
    Parts: PartInstance list
    Joints: Joint list
    Gears: GearMesh list
    Drives: Drive list
}

module Platform =
    let build (genome: Genome) stations phase =
        let parts, joints, gears, drives = ResizeArray<PartInstance>(), ResizeArray<Joint>(), ResizeArray<GearMesh>(), ResizeArray<Drive>()
        let at = Vec3.create
        let zAxis = at 0 0 1
        let structural id part color pos rot body =
            parts.Add { Id = id; Part = part; Color = color; Position = pos; Rotation = rot; Body = Some body }
        let connector id part pos rot =
            parts.Add { Id = id; Part = part; Color = 8; Position = pos; Rotation = rot; Body = None }
        let joint id a b pos source =
            joints.Add { Id = id; BodyA = a; BodyB = b; Position = pos; Axis = zAxis; Connector = source }
        let link id part body color start finish localStart =
            let rot = Rotation.beamWithNormal zAxis (Vec3.subtract finish start)
            structural id part color (Vec3.subtract start (Rotation.apply rot (at 0 0 localStart))) rot body
        if stations < 1 || stations > 3 then invalidArg "stations" "The rail platform supports one to three output stations."
        let driveX = 40.0
        let lastShaft = 2 + (stations-1)*4
        let lastAxis = driveX + float lastShaft*60.0
        let segmentCount = int (ceil ((lastAxis + 120.0)/320.0))
        for sideZ in [0.0; 120.0] do
            for level in 0..1 do
                for segment in 0..segmentCount-1 do
                    structural $"rail-{sideZ}-{level}-{segment}" "3703.dat" 7
                        (at (140.0 + float segment*320.0 + float level*40.0) (-10.0-float level*24.0) sideZ) Rotation.identity "chassis"
        let plateRot = { X = at 0 0 1; Y = at 0 1 0; Z = at -1 0 0 }
        let bridgeXs = [
            yield 30.0
            for n in 1..segmentCount-1 do yield float n*320.0-70.0
            yield float segmentCount*320.0-50.0
        ]
        for n, x in bridgeXs |> List.indexed do
            structural $"bridge-{n}" "3460.dat" 7 (at x -42 70) plateRot "chassis"
        structural "motor" "58121.dat" 7 (at driveX 0 150) Rotation.identity "chassis"
        link "motor-spacer" "32523.dat" "chassis" 7 (at (driveX-20.0) 0 140) (at (driveX+20.0) 0 140) -20.0
        for x in [driveX-20.0; driveX+20.0] do
            connector $"motor-mount-{x}" "6558.dat" (at x 0 140) (Rotation.shaft 0)
        for index in 0..lastShaft do
            let x = driveX + float index*60.0
            let body = $"shaft-{index}"
            let gearAngle = if index % 2 = 0 then phase else Math.PI/24.0-phase
            structural $"gear-{index}" "3648b.dat" 4 (at x 0 60) (Rotation.aroundZ gearAngle) body
            let axlePart, axleZ =
                if index = 0 then "3707.dat", 90.0
                elif index >= 2 && (index-2) % 4 = 0 then "3707.dat", 60.0
                else "44294.dat", 60.0
            connector $"axle-{index}" axlePart (at x 0 axleZ) (Rotation.shaft gearAngle)
            joint $"bearing-{index}" "chassis" body (at x 0 axleZ) $"axle-{index}"
            if index > 0 then
                for bushZ in [20.0; 100.0] do
                    structural $"bush-{index}-{bushZ}" "3713.dat" 8 (at x 0 bushZ) (Rotation.aroundZ gearAngle) body
            if index > 0 then gears.Add { BodyA = $"shaft-{index-1}"; BodyB = body; Ratio = 1.0 }
        drives.Add { MotorPart = "motor"; Joint = "bearing-0"; Speed = 3.0*float genome.MotorDirection; MaxTorque = 0.4 }
        if genome.TowWagon then
            let wagonX x = x - 60.0
            for z in [0.0; 120.0] do
                for level in 0..1 do
                    structural $"wagon-rail-{z}-{level}" "3703.dat" 7
                        (at (wagonX (-180.0+float level*40.0)) (-10.0-float level*24.0) z) Rotation.identity "wagon-chassis"
            for x in [-290.0; -50.0] do
                structural $"wagon-bridge-{x}" "3460.dat" 7 (at (wagonX x) -42 70) plateRot "wagon-chassis"
            link "drawbar-beam" "32063.dat" "drawbar" 7 (at 0 0 -25) (at -100 0 -25) -50
            for id, x, body in ["walker", 0.0, "chassis"; "wagon", -100.0, "wagon-chassis"] do
                let pin = $"drawbar-{id}-pin"
                let rotation = { X = at 0 0 -1; Y = at 0 1 0; Z = at 1 0 0 }
                connector pin "3673.dat" (at x 0 -15) rotation
                joint $"drawbar-{id}-pitch" body "drawbar" (at x 0 -15) pin
            let axleY = float genome.WagonAxleHole*20.0
            for index in 0..1 do
                let front = index = 0
                let body = $"wagon-wheel-{index}"
                let planes = if front then [-15.0; 25.0] else [95.0; 135.0]
                for z in planes do
                    link $"wagon-bearing-{z}" "32063.dat" "wagon-chassis" 7
                        (at (wagonX -160) 0 z) (at (wagonX -160) 100 z) -50
                let frameZ = if front then 0.0 else 120.0
                connector $"wagon-bearing-pin-{index}" "6558.dat" (at (wagonX -160) 0 frameZ) (Rotation.shaft 0)
                let wheelZ = if front then -60.0 else 180.0
                structural body "56145c01.dat" 8 (at (wagonX -160) axleY wheelZ) Rotation.identity body
                let axleZ = if front then -22.0 else 142.0
                let axleId = $"wagon-axle-{index}"
                connector axleId "3707.dat" (at (wagonX -160) axleY axleZ) (Rotation.shaft 0)
                for z in (if front then [-90.0; -25.0; 35.0] else [85.0; 145.0; 210.0]) do
                    structural $"wagon-bush-{z}" "4265c.dat" 7 (at (wagonX -160) axleY z) Rotation.identity body
                joint $"wagon-bearing-{index}" "wagon-chassis" body (at (wagonX -160) axleY planes.Head) axleId
        if genome.Remote then
            let batteryBody = if genome.BatteryOnWagon then "wagon-chassis" else "chassis"
            let receiverBody = if genome.ReceiverOnWagon then "wagon-chassis" else "chassis"
            let batteryX =
                if genome.BatteryOnWagon then -240.0+float (genome.BatterySlot-1)*40.0
                else float segmentCount*160.0-20.0 + float (genome.BatterySlot-1)*float stations*60.0
            let topLevel = 5 + genome.BatteryLevel
            let batteryY = -24.0*float topLevel
            for z in [0.0; 120.0] do
                for level in 2..topLevel do
                    structural $"battery-support-{z}-{level}" "3703.dat" 7
                        (at batteryX (-10.0-float level*24.0) z) Rotation.identity batteryBody
            structural "payload-battery" "58119.dat" 7 (at batteryX batteryY 60) Rotation.identity batteryBody
            for x in [batteryX-100.0; batteryX+100.0] do
                for z in [20.0; 100.0] do
                    connector $"battery-mount-{x}-{z}" "6558.dat" (at x batteryY z) (Rotation.shaft 0)
            let receiverX =
                if genome.ReceiverOnWagon then -300.0+float genome.ReceiverSlot*80.0
                else 80.0 + float genome.ReceiverSlot*(float segmentCount*320.0-160.0)/2.0
            let receiverZ = if genome.ReceiverSide = 0 then -60.0 else 180.0
            let receiverRotation =
                if genome.ReceiverSide = 0 then { X = at -1 0 0; Y = at 0 1 0; Z = at 0 0 -1 }
                else Rotation.identity
            structural "payload-receiver" "58123.dat" 7 (at receiverX -34 receiverZ) receiverRotation receiverBody
            for x in [receiverX-20.0; receiverX+20.0] do
                let pinZ = if genome.ReceiverSide = 0 then -20.0 else 140.0
                connector $"receiver-mount-{x}" "6558.dat" (at x -24 pinZ) (Rotation.shaft 0)
        { Parts = List.ofSeq parts; Joints = List.ofSeq joints; Gears = List.ofSeq gears; Drives = List.ofSeq drives }
