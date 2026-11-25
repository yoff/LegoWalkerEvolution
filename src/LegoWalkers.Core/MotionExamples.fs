namespace LegoWalkers

open System

module MotionExamples =
    let walker1Source =
        "https://github.com/yoff/LegoTechnicSimulation/blob/7db4596fe7f970fadc7b53893be32b4b93249d9e/sample_models/Walker1/Walker1.ldr"
    let private point x y = Vec3.create x y 0
    let private hinge id a sa b sb = Motion.couple id (Motion.port a sa) (Motion.port b sb)

    let fourBar (g: Genome) angle =
        let d, c, r = float g.GroundHoles*20.0, float g.CouplerHoles*20.0, float g.RockerHoles*20.0
        let b = point (20.0*cos angle) (20.0*sin angle)
        let delta = Vec3.subtract (point d 0) b
        let distance = Vec3.length delta
        if not (Double.IsFinite angle) || distance < 1e-9 then
            Error "Invalid crank angle or coincident circle centers."
        elif distance >= c+r-1e-6 || distance <= abs(c-r)+1e-6 then
            Error "The four-bar cannot close without reaching a toggle singularity."
        else
            // An exact assembly seed preserves the existing four-bar's branch and geometry.
            let u = Vec3.scale (1.0/distance) delta
            let along = (c*c-r*r+distance*distance)/(2.0*distance)
            let height = sqrt (c*c-along*along)
            let joint = Vec3.add b (Vec3.add (Vec3.scale along u) (Vec3.scale height (point -u.Y u.X)))
            let bearing (start: Vec3) (finish: Vec3) = atan2 (finish.Y-start.Y) (finish.X-start.X)
            Ok {
                Bodies = [
                    Motion.ground "ground" [Motion.site "drive" 0 0; Motion.site "rocker" d 0]
                    Motion.rigid "crank" 0 0 angle [Motion.site "root" 0 0; Motion.site "tip" 20 0; Motion.site "end" 40 0]
                    Motion.rigid "coupler" b.X b.Y (bearing b joint)
                        [Motion.site "root" 0 0; Motion.site "joint" c 0; Motion.site "foot" 100 0]
                    Motion.rigid "rocker" d 0 (bearing (point d 0) joint)
                        [Motion.site "root" 0 0; Motion.site "joint" r 0; Motion.site "end" 100 0]
                ]
                Hinges = [
                    hinge "drive" "ground" "drive" "crank" "root"
                    hinge "crank-coupler" "crank" "tip" "coupler" "root"
                    hinge "coupler-rocker" "coupler" "joint" "rocker" "joint"
                    hinge "rocker-ground" "rocker" "root" "ground" "rocker"
                ]
                Motor = { Body = "crank"; Phase = 0.0 }
                Transmissions = []
                Feet = [Motion.foot "coupler-tip" "coupler" "foot" 4]
            }

    // Front-left leg: exact hole spans replace the source's rounded assembly placements.
    let walker1 = {
        Bodies = [
            Motion.ground "ground" [Motion.site "drive" -100 0; Motion.site "upper" -60 -80]
            Motion.rigid "crank" -100 0 0 [Motion.site "root" 0 0; Motion.site "tip" 20 0]
            Motion.rigid "upper" -60 -80 Math.PI [Motion.site "root" 0 0; Motion.site "tip" 40 0]
            Motion.rigid "crossbar" -120 0 0
                [Motion.site "left" 0 0; Motion.site "drive" 40 0; Motion.site "right" 80 0]
            Motion.rigid "brace" -60 -80 (atan2 80.0 20.0) [Motion.site "root" 0 0; Motion.site "tip" 80 0]
            Motion.rigid "long-leg" -100 -80 (atan2 196.0 52.0)
                [Motion.site "root" 0 0; Motion.site "drive" 80 0; Motion.site "tip" 200 0]
            Motion.rigid "short-leg" -120 0 (atan2 116.0 32.0) [Motion.site "root" 0 0; Motion.site "tip" 120 0]
            Motion.rigid "sole" -88 116 0
                [Motion.site "left" 0 0; Motion.site "right" 40 0; Motion.site "contact" 20 14]
        ]
        Hinges = [
            hinge "drive" "ground" "drive" "crank" "root"
            hinge "drive-crossbar" "crank" "tip" "crossbar" "drive"
            hinge "upper-ground" "ground" "upper" "upper" "root"
            hinge "shared-upper-pivot" "upper" "root" "brace" "root"
            hinge "brace-crossbar" "brace" "tip" "crossbar" "right"
            hinge "drive-long-leg" "crank" "tip" "long-leg" "drive"
            hinge "upper-long-leg" "upper" "tip" "long-leg" "root"
            hinge "crossbar-short-leg" "crossbar" "left" "short-leg" "root"
            hinge "long-leg-sole" "long-leg" "tip" "sole" "right"
            hinge "short-leg-sole" "short-leg" "tip" "sole" "left"
        ]
        Motor = { Body = "crank"; Phase = 0.0 }
        Transmissions = []
        Feet = [Motion.foot "sole" "sole" "contact" 4]
    }

    /// Holdout construction example; not a founder or an enabled evolutionary mutation.
    let doubleParallelogram height toe =
        let baseProgram = {
            Bodies = [
                Motion.ground "chassis" [Motion.site "lower" -80 0; Motion.site "upper" -80 -height; Motion.site "drive" -20 80]
                Motion.rigid "crank" -20 80 0 [Motion.site "root" 0 0; Motion.site "tip" 20 0]
                Motion.rigid "lower-arm" -80 0 0 [Motion.site "root" 0 0; Motion.site "tip" 80 0]
                Motion.rigid "driven-leg" 0 0 (Math.PI/2.0)
                    [Motion.site "top" 0 0; Motion.site "drive" 80 0; Motion.site "bottom" 200 0]
            ]
            Hinges = [
                hinge "bearing" "chassis" "drive" "crank" "root"
                hinge "crank-mid-leg" "crank" "tip" "driven-leg" "drive"
                hinge "lower-bearing" "chassis" "lower" "lower-arm" "root"
                hinge "shared-corner-a" "lower-arm" "tip" "driven-leg" "top"
            ]
            Motor={Body="crank"; Phase=0}; Transmissions=[]
            Feet=[Motion.foot "foot" "driven-leg" "bottom" 4]
        }
        baseProgram
        |> Motion.addParallelogram "upper"
            (Motion.port "chassis" "lower", Motion.port "chassis" "upper")
            (Motion.port "lower-arm" "root", Motion.port "lower-arm" "tip")
            ("corner","top") [Motion.site "toe-link" toe 0]
        |> Result.bind (Motion.addHingedFoot "lower"
            (Motion.port "upper-output" "corner", Motion.port "upper-output" "toe-link")
            (Motion.port "driven-leg" "top", Motion.port "driven-leg" "bottom")
            ("left","right") (Motion.site "contact" (toe/2.0) 14) "foot")
        |> Result.map (fun program ->
            let names = Map.ofList [
                "upper-parallel","upper-arm"; "upper-output","upright-l"
                "lower-parallel","parallel-leg"; "lower-output","shoe"
            ]
            let hingeNames = Map.ofList [
                "upper-base","upper-bearing"; "upper-tip","upper-upright"; "upper-driver","shared-corner-b"
                "lower-base","parallel-top"; "lower-tip","parallel-shoe"; "lower-driver","driven-shoe"
            ]
            let bodyName id = Map.tryFind id names |> Option.defaultValue id
            let siteName body site =
                match body,site with
                | "lower-parallel","root" -> "top"
                | "lower-parallel","tip" -> "bottom"
                | _ -> site
            let port (p: MotionPort) = {Body=bodyName p.Body; Site=siteName p.Body p.Site}
            { program with
                Bodies=program.Bodies |> List.map (fun b ->
                    {b with Id=bodyName b.Id; Sites=b.Sites |> List.map (fun s -> {s with Id=siteName b.Id s.Id})})
                Hinges=program.Hinges |> List.map (fun h ->
                    {h with Id=Map.tryFind h.Id hingeNames |> Option.defaultValue h.Id; A=port h.A; B=port h.B})
                Feet=program.Feet |> List.map (fun f -> {f with Port=port f.Port}) })

    /// Canonical proportions, not Technic hole-grid dimensions: https://en.wikipedia.org/wiki/Jansen%27s_linkage
    let jansen =
        // Circle intersections select only the initial assembly branch; Motion solves every subsequent pose.
        let meet (p: Vec3) (r: float) (q: Vec3) (s: float) =
            let delta = Vec3.subtract q p
            let distance = Vec3.length delta
            let along = (r*r-s*s+distance*distance)/(2.0*distance)
            let height = sqrt (r*r-along*along)
            let axis = Vec3.scale (1.0/distance) delta
            let center = Vec3.add p (Vec3.scale along axis)
            let offset = Vec3.scale height (point -axis.Y axis.X)
            [Vec3.add center offset; Vec3.subtract center offset]
        let o,y,z = point 0 0,point -38 -7.8,point 15 0
        let x = meet z 50.0 y 41.5 |> List.maxBy _.Y
        let w = meet z 61.9 y 39.3 |> List.minBy _.Y
        let v = meet y 40.1 x 55.8 |> List.minBy _.X
        let u = meet v 39.4 w 36.7 |> List.minBy _.X
        let t = meet w 49.0 u 65.7 |> List.minBy _.Y
        let downward (p: Vec3) = point p.X -p.Y
        let body id (origin: Vec3) sites =
            let origin = downward origin
            Motion.rigid id origin.X origin.Y 0
                (sites |> List.map (fun (name,p) ->
                    {Id=name; Position=Vec3.subtract (downward p) origin}))
        {
            Bodies = [
                Motion.ground "ground" [Motion.site "drive" 0 0; Motion.site "rocker" -38 7.8]
                body "crank" o ["root",o; "tip",z]
                body "upper-drive" z ["crank",z; "rocker",x]
                body "lower-drive" z ["crank",z; "rocker",w]
                body "upper-triangle" y ["pivot",y; "drive",x; "knee",v]
                body "rocker" y ["root",y; "tip",w]
                body "shin" v ["upper",v; "lower",u]
                body "foot-triangle" w ["pivot",w; "knee",u; "toe",t]
            ]
            Hinges = [
                hinge "drive-bearing" "ground" "drive" "crank" "root"
                hinge "crank-upper" "crank" "tip" "upper-drive" "crank"
                hinge "crank-lower" "crank" "tip" "lower-drive" "crank"
                hinge "upper-drive" "upper-drive" "rocker" "upper-triangle" "drive"
                hinge "upper-bearing" "ground" "rocker" "upper-triangle" "pivot"
                hinge "rocker-bearing" "ground" "rocker" "rocker" "root"
                hinge "lower-drive" "lower-drive" "rocker" "foot-triangle" "pivot"
                hinge "lower-rocker" "rocker" "tip" "foot-triangle" "pivot"
                hinge "upper-knee" "upper-triangle" "knee" "shin" "upper"
                hinge "lower-knee" "shin" "lower" "foot-triangle" "knee"
            ]
            Motor = {Body="crank"; Phase=0}
            Transmissions = []
            Feet = [Motion.foot "toe" "foot-triangle" "toe" 1]
        }

    let names = ["four-bar"; "walker1"; "jansen"]
    let named = function
        | "four-bar" -> fourBar Genome.defaults 0
        | "walker1" -> Ok walker1
        | "jansen" -> Ok jansen
        | name -> Error $"Unknown motion example {name}; expected four-bar, walker1 or jansen."
