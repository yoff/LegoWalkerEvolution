namespace LegoWalkers

open System

module WalkerFootPlates =
    let version = "stud-mounted-foot-plates-v3"

    let private addSelected (s: ScreenedMotion) (g: WalkerGenome) layers selected =
        try
            if layers<1 || layers>2 then
                invalidArg "layers" "Foot soles support one or two plate layers."
            let contacts = g.Feet |> List.filter (fun foot -> Set.contains foot.Foot selected)
                                 |> List.collect WalkerFootChoice.contacts
            let plates =
                [for contact in contacts do
                    let body =
                        g.Bodies |> List.tryFind (fun body -> body.Parts |> List.exists (fun p -> p.Id=contact.Part))
                        |> Option.defaultWith (fun () -> invalidArg "foot" $"Missing sole {contact.Part}.")
                    let sole = body.Parts |> List.find (fun p -> p.Id=contact.Part)
                    if sole.Part<>"3701.dat" || abs sole.Rotation.Z.Z<0.999999 then
                        invalidArg "foot" "Foot plates require an existing transverse 3701 sole."
                    for layer in 1..layers do
                        yield body.Body,sole.Id,layer,
                            {sole with Id = $"{sole.Id}-plate-{layer}";Part="3710.dat";Color=2
                                       Position=Vec3.add sole.Position (Vec3.scale (24.0+8.0*float(layer-1)) sole.Rotation.Y)}]
            let plateId part =
                plates |> List.find (fun (_,id,layer,_) -> id=part && layer=layers) |> fun (_,_,_,plate) -> plate.Id
            let candidate = {
                g with
                    Bodies=g.Bodies |> List.map (fun body ->
                        let additions = plates |> List.choose (fun (id,_,_,plate) ->
                            if id=body.Body then Some plate else None)
                        {body with Parts=body.Parts @ additions})
                    Feet=g.Feet |> List.map (fun foot ->
                        if not(Set.contains foot.Foot selected) then foot else
                            {foot with Part=plateId foot.Part
                                       AdditionalContacts=foot.AdditionalContacts |> Option.map (List.map (fun contact ->
                                           {contact with Part=plateId contact.Part}))})
            }
            WalkerRealization.compile s candidate |> Result.map _.Genome
        with :? ArgumentException as error ->
            Error {Gate="lego-realizability";Outcome="reject";Code="unsupported-foot-plates";Diagnostics=[error.Message]}

    let add s g layers = addSelected s g layers (g.Feet |> List.map _.Foot |> Set.ofList)

    let private brickUnderPlate (parts: Map<string,WalkerPlacement>) (plate: WalkerPlacement) =
        [1;2] |> List.tryPick (fun layer ->
            let suffix = $"-plate-{layer}"
            if not(plate.Id.EndsWith suffix) then None else
            let id = plate.Id.Substring(0,plate.Id.Length-suffix.Length)
            parts |> Map.tryFind id |> Option.filter (fun brick ->
                brick.Part="3701.dat" && brick.Rotation=plate.Rotation
                && Vec3.distance plate.Position
                    (Vec3.add brick.Position (Vec3.scale (24.0+8.0*float(layer-1)) brick.Rotation.Y))<1e-6)
            |> Option.map (fun brick -> brick,layer))

    let prepareWithLayers layers (s: ScreenedMotion) (g: WalkerGenome) =
        if layers<1 || layers>2 then invalidArg "layers" "Foot soles support one or two plate layers."
        WalkerRealization.compile s g |> Result.bind (fun _ ->
            let originalParts = g.Bodies |> List.collect _.Parts |> List.map (fun p -> p.Id,p) |> Map.ofList
            let singles = g.Feet |> List.choose (fun foot ->
                let part = originalParts[foot.Part]
                if (WalkerFootChoice.contacts foot).Length<>1 || part.Part<>"3710.dat" then None else
                brickUnderPlate originalParts part |> Option.map (fun (brick,count) -> foot.Foot,brick.Id,count))
            let replaced = singles |> List.map (fun (foot,brick,_) -> foot,brick) |> Map.ofList
            let removed = singles |> List.collect (fun (_,brick,count) ->
                [for layer in 1..count -> $"{brick}-plate-{layer}"]) |> Set.ofList
            let source =
                if singles.IsEmpty then g else
                {g with Bodies=g.Bodies |> List.map (fun body ->
                            {body with Parts=body.Parts |> List.filter (fun p -> not(removed.Contains p.Id))})
                        Feet=g.Feet |> List.map (fun foot ->
                            match replaced |> Map.tryFind foot.Foot with
                            | Some brick -> {foot with Part=brick}
                            | None -> foot)}
            WalkerInnerFeet.ensure s source |> Result.bind (fun paired ->
                let parts = paired.Bodies |> List.collect _.Parts |> List.map (fun p -> p.Id,p) |> Map.ofList
                let brickFeet = paired.Feet |> List.filter (fun foot -> parts[foot.Part].Part="3701.dat")
                let selected = brickFeet |> List.map _.Foot |> Set.ofList
                let plated = if selected.IsEmpty then Ok paired else addSelected s paired layers selected
                let extended = plated |> Result.bind (fun genome ->
                    if layers=1 then Ok genome else
                    let physical = genome.Bodies |> List.collect (fun body ->
                        body.Parts |> List.map (fun p -> p.Id,(body.Body,p))) |> Map.ofList
                    let allParts = physical |> Map.map (fun _ (_,p) -> p)
                    let additions = genome.Feet |> List.collect WalkerFootChoice.contacts |> List.choose (fun contact ->
                        let owner,part = physical[contact.Part]
                        if part.Part<>"3710.dat" then None else
                        match brickUnderPlate allParts part with
                        | Some(brick,1) ->
                            let plate = {part with
                                            Id=brick.Id+"-plate-2"
                                            Position=Vec3.add part.Position (Vec3.scale 8.0 part.Rotation.Y)}
                            Some(owner,part.Id,plate)
                        | Some(_,2) -> None
                        | _ -> invalidArg "foot" "A second plate layer requires the supported stud-mounted brick sole.")
                    let lower = additions |> List.map (fun (_,id,part) -> id,part.Id) |> Map.ofList
                    let contact part = lower |> Map.tryFind part |> Option.defaultValue part
                    if additions.IsEmpty then Ok genome else
                    let changed = {genome with
                                    Bodies=genome.Bodies |> List.map (fun body ->
                                        let extra = additions |> List.choose (fun (owner,_,part) ->
                                            if owner=body.Body then Some part else None)
                                        {body with Parts=body.Parts @ extra})
                                    Feet=genome.Feet |> List.map (fun foot ->
                                        {foot with Part=contact foot.Part
                                                   AdditionalContacts=foot.AdditionalContacts |> Option.map (List.map (fun c ->
                                                       {c with Part=contact c.Part}))})}
                    WalkerRealization.compile s changed |> Result.map _.Genome)
                extended |> Result.map (fun genome ->
                    let unsupported = genome.Feet |> List.filter (fun foot ->
                        let part = genome.Bodies |> List.collect _.Parts |> List.find (fun p -> p.Id=foot.Part)
                        part.Part<>"3710.dat" || (WalkerFootChoice.contacts foot).Length<>2)
                    genome,[{Gate="foot-construction";Outcome=if unsupported.IsEmpty then "pass" else "warning"
                             Code="paired-stock-plate-soles"
                             Diagnostics=[$"Supported transverse brick ankles use two parallel soles and {layers} plate layer(s); complete existing stacks are retained."
                                          "Unsupported non-brick foot forms are retained explicitly, not reclassified as plate soles."]
                                         @ (unsupported |> List.map (fun f -> $"Foot {f.Foot}: unsupported sole construction retained."))}])))

    let prepare s g = prepareWithLayers 1 s g
