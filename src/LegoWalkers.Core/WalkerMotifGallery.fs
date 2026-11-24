namespace LegoWalkers

open System.IO

module WalkerMotifGallery =
    let write directory =
        if Directory.Exists directory && not (Directory.EnumerateFileSystemEntries directory |> Seq.isEmpty) then
            invalidArg "output" "Output directory must be empty."
        let unwrap result = Result.defaultWith (Realization.json >> invalidOp) result
        let program, genome = WalkerSeeds.fourBar 2
        let screened = Realization.screen program 72 |> unwrap
        let original = WalkerRealization.compile screened genome |> unwrap
        let pin = original.Genome.Connections |> List.find (fun c -> c.Group = "leg-0-0-crank")
        let longGenome =
            WalkerConstruction.choices screened original.Genome |> Seq.find (fun choice ->
                choice.Pivot = pin.Group && choice.Motif = "smooth-pivot-longer-pin"
                && (choice.Genome.Connections |> List.exists (fun c ->
                    c.Group = pin.Group && c.Placement.Part = "3673.dat"
                    && c.Placement.Rotation = pin.Placement.Rotation)))
            |> _.Genome
        let shorter = WalkerConstruction.choices screened longGenome |> Seq.find (fun c ->
            c.Pivot = pin.Group && c.Motif = "smooth-pivot-shorter-pin")
        let reversed = WalkerConstruction.choices screened original.Genome |> Seq.find (fun c ->
            c.Pivot = pin.Group && c.Motif = "smooth-pivot-reversed-pin")
        let body = original.Genome.Bodies |> List.find (fun b -> b.Body = "leg-0-1-coupler")
        let layer = WalkerConstruction.repairs screened original.Genome
                        { Kind = "rigid-body-partition"; Parts = [body.Parts.Head.Id]; Bodies = ["shaft-2"] } 1
                    |> List.exactlyOne
        let witnesses = [
            "smooth-passive-pivot", "shorter-pin", "Shorter compatible pin",
                "40 to 30 LDU shaft; same midpoint, declared members and motion.", longGenome, shorter
            "smooth-passive-pivot", "reversed-pin", "Reverse pin orientation",
                "Same shaft interval and midpoint; the local pin axis reverses.", original.Genome, reversed
            "passive-link-layer-clearance", "layer-clearance", "Move a passive link outward",
                "+10 LDU depth; existing pins and planar motion stay unchanged.", original.Genome, layer
        ]
        if Set.ofList (witnesses |> List.map (fun (motif,_,_,_,_,_) -> motif))
           <> Set.ofList (WalkerBuilder.catalogue |> List.map _.Id) then
            invalidOp "Every catalogue motif requires a construction-gallery witness."
        Directory.CreateDirectory directory |> ignore
        WalkerBuilder.write directory WalkerBuilder.defaultPolicy
        let examples = witnesses |> List.map (fun (motif,id,title,caption,before,choice) ->
            let parent = WalkerRealization.compile screened before |> unwrap
            let child = WalkerRealization.compile screened choice.Genome |> unwrap
            let pivots =
                if choice.Pivot <> "" then before.Connections |> List.filter (fun c -> c.Group = choice.Pivot)
                else
                    let hinges = screened.Program.Hinges |> List.filter (fun h ->
                        List.contains h.A.Body choice.AffectedBodies || List.contains h.B.Body choice.AffectedBodies)
                    before.Connections |> List.filter (fun c -> hinges |> List.exists (fun h -> List.contains h.Id c.Hinges))
            let selectedPins = pivots |> List.map _.Placement
            let focus (g: WalkerGenome) =
                g.Bodies |> List.collect _.Parts
                |> List.filter (fun part -> selectedPins |> List.exists (fun p -> WalkerStructural.engaged p part))
                |> List.map _.Id
            let ids = Set.ofList (focus before @ focus choice.Genome @ (selectedPins |> List.map _.Id))
            let parts robot = robot.Assembly.Parts |> List.filter (fun p -> ids.Contains p.Id)
            let beforeParts, afterParts = parts parent, parts child
            let changed = List.zip beforeParts afterParts |> List.choose (fun (a,b) -> if a <> b then Some a.Id else None)
            let schematic (part: PartInstance) =
                let world local = Vec3.add part.Position (Rotation.apply part.Rotation local)
                let shaft =
                    if selectedPins |> List.exists (fun pin -> pin.Id = part.Id) then
                        let low,high = Catalog.shaftRange part.Part
                        [world (Vec3.create low 0 0); world (Vec3.create high 0 0)]
                    else []
                {| id = part.Id; part = part.Part; changed = List.contains part.Id changed
                   ports = (Catalog.get part.Part).Ports |> List.map (fun port -> world port.Position)
                   shaft = shaft |}
            let target = Path.Combine(directory, id)
            WalkerClearance.writeRealization (Path.Combine(target, "before")) screened parent |> ignore
            WalkerClearance.writeRealization (Path.Combine(target, "after")) screened child |> ignore
            WalkerConstruction.write (Path.Combine(target, "after")) choice
            {| id = id; motif = motif; title = title; caption = caption
               mechanismId = screened.MechanismId; beforeGenotypeId = parent.GenotypeId; afterGenotypeId = child.GenotypeId
               before = beforeParts |> List.map schematic; after = afterParts |> List.map schematic |})
        File.WriteAllText(Path.Combine(directory, "gallery.json"),
                          Realization.json {| schemaVersion = 1; mode = "construction-schematic-not-dynamics"; examples = examples |}+"\n")
