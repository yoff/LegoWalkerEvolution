namespace LegoWalkers

open System.Collections.Generic

module WalkerScaffold =
    let maximumAlternatives = 12

    let private sockets (part: WalkerPlacement) =
        (Catalog.get part.Part).Ports |> List.choose (fun port ->
            let axis = Rotation.apply part.Rotation port.Axis
            if port.Kind=RoundHole && abs axis.Z>0.999 then
                Some(Vec3.add part.Position (Rotation.apply part.Rotation port.Position))
            else None)

    let balanceForSource (request: WalkerPackagingRequest) (source: WalkerGenome) =
        request.Balance |> Option.map (fun model ->
            let original = (request.Source.Bodies |> List.collect _.Parts) @ request.Source.Mounts
                           |> List.map (fun p -> p.Id,p) |> Map.ofList
            let selected = (source.Bodies |> List.collect _.Parts) @ source.Mounts
                           |> List.map (fun p -> p.Id,p) |> Map.ofList
            let mass,moment =
                Option.defaultValue [] model.ScaffoldComponents
                |> List.fold (fun (total,moment) measured ->
                    match Map.tryFind measured.Part selected with
                    | None -> total-measured.MassKg,Vec3.subtract moment (Vec3.scale measured.MassKg measured.CenterLdu)
                    | Some placed ->
                        let previous = original[measured.Part]
                        let offset = Vec3.subtract measured.CenterLdu previous.Position
                        let local = Vec3.create (Vec3.dot offset previous.Rotation.X)
                                               (Vec3.dot offset previous.Rotation.Y) (Vec3.dot offset previous.Rotation.Z)
                        let moved = Vec3.add placed.Position (Rotation.apply placed.Rotation local)
                        total,Vec3.add moment (Vec3.scale measured.MassKg (Vec3.subtract moved measured.CenterLdu)))
                    (model.TotalMassKg,Vec3.scale model.TotalMassKg model.CenterOfMassLdu)
            {model with TotalMassKg=mass;CenterOfMassLdu=Vec3.scale (1.0/mass) moment
                        ScaffoldComponents=model.ScaffoldComponents |> Option.map (List.filter (fun c -> selected.ContainsKey c.Part))})

    let optionsObserved inspect (request: WalkerPackagingRequest) screened =
        let ids =
            (request.Source.Bodies |> List.collect _.Parts) @ request.Source.Mounts
            @ (request.Source.Connections |> List.map _.Placement) |> List.map _.Id
        let namespacePrefix =
            Seq.initInfinite (fun index -> if index=0 then "scaffold-link" else $"scaffold-link-{index}")
            |> Seq.find (fun prefix -> ids |> List.forall (fun id -> not(id.StartsWith(prefix+"-"))))
        let ground = request.Program.Bodies |> List.find _.Grounded |> _.Id
        let frame,removed = WalkerPackaging.scaffold request
        let sourceGround = request.Source.Bodies |> List.find (fun b -> b.Body=ground)
        let retainedMounts = request.Source.Mounts |> List.filter (fun p -> not(removed.Contains p.Id))
        let driven = request.Program.Motor.Body::(request.Program.Transmissions |> List.collect (fun t -> [t.Input;t.Output])) |> Set.ofList
        let retainedJoint id =
            request.Program.Hinges |> List.exists (fun h ->
                h.Id=id && ((h.A.Body=ground && not(driven.Contains h.B.Body))
                            || (h.B.Body=ground && not(driven.Contains h.A.Body))))
        let interfaces =
            request.Source.Connections |> List.filter (fun c ->
                not(removed.Contains c.Placement.Id) && List.exists retainedJoint c.Hinges)
            |> List.map _.Placement
        let fixedParts,backbone = frame |> List.partition (fun p -> interfaces |> List.exists (fun pin -> WalkerStructural.engaged pin p))
        let attachments = retainedMounts |> List.choose (fun pin ->
            let fixedSocket = fixedParts |> List.tryPick (fun part ->
                WalkerStructural.engagedPorts pin part |> List.tryPick (fun (port,point) ->
                    if port.Kind=RoundHole && abs((Rotation.apply part.Rotation port.Axis).Z)>0.999 then Some point else None))
            if backbone |> List.exists (WalkerStructural.engaged pin) then
                fixedSocket |> Option.map (fun socket ->
                    let part = fixedParts |> List.find (WalkerStructural.engaged pin)
                    let available = sockets part |> List.filter (fun point ->
                        interfaces |> List.forall (fun pivot ->
                            WalkerStructural.engagedPorts pivot part |> List.forall (fun (_,occupied) ->
                                Vec3.distance point occupied>1e-6)))
                    pin.Id,(socket::available |> List.distinct))
            else None)
        let walking = {request with Drive={request.Drive with Gears=[]}}
        WalkerPackaging.motionSpace walking screened |> Result.map (fun space ->
            let checkedParts = Dictionary<WalkerPlacement,bool>()
            let mutable examined,rejected,hits = 0,0,0
            let clear part =
                match checkedParts.TryGetValue part with
                | true,accepted -> hits<-hits+1;accepted
                | _ ->
                    let accepted = WalkerMotionSpace.fits space ground [part]
                    checkedParts.Add(part,accepted)
                    examined<-examined+1
                    if not accepted then rejected<-rejected+1
                    accepted
            let shifts =
                [Vec3.create 0 -24 0;Vec3.create 0 -48 0;Vec3.create 0 -72 0
                 Vec3.create -40 -24 0;Vec3.create 40 -24 0
                 Vec3.create 0 -24 -40;Vec3.create 0 -24 40]
            let alternatives = seq {
                if not backbone.IsEmpty && not fixedParts.IsEmpty && not attachments.IsEmpty then
                    for delta in shifts do
                        for side in [1;-1] do
                            let moved = backbone |> List.map (fun p -> {p with Position=Vec3.add p.Position delta})
                            let oldAttachments = attachments |> List.map fst |> Set.ofList
                            let mounts = retainedMounts |> List.filter (fun p -> not(oldAttachments.Contains p.Id))
                                         |> List.map (fun p ->
                                             if backbone |> List.exists (WalkerStructural.engaged p) then
                                                 {p with Position=Vec3.add p.Position delta}
                                             else p)
                            let anchors = moved |> List.collect sockets |> List.distinct
                            let parts,pins = ResizeArray<WalkerPlacement>(),ResizeArray<WalkerPlacement>()
                            let mutable possible = (moved @ fixedParts @ mounts) |> List.forall clear
                            for index,(_,socketChoices) in List.indexed attachments do
                                if possible then
                                    let prefix = $"{namespacePrefix}-{index}"
                                    let fixedStock = moved @ fixedParts @ List.ofSeq parts
                                    let occupied = mounts @ List.ofSeq pins @ interfaces
                                    let accepts (beams,connectors) =
                                        (beams @ connectors) |> List.forall (fun p ->
                                            clear p && WalkerMotionSpace.fitsFixed [p] (fixedStock @ occupied))
                                    let route =
                                        socketChoices |> List.sortBy (fun socket ->
                                            anchors |> List.map (Vec3.distance socket) |> List.min)
                                        |> List.tryPick (fun socket ->
                                            let center = request.Source.Feet |> List.averageBy _.Depth
                                            let plane = socket.Z+(if socket.Z<center then -20.0 else 20.0)
                                            WalkerPackaging.motionSupportRoutesWithPruning accepts side prefix anchors occupied socket plane
                                            |> List.tryPick (fun (beams,connectors) ->
                                                match WalkerPackaging.pin (prefix+"-mount-pin") socket {socket with Z=plane} with
                                                | Error _ -> None
                                                | Ok mount ->
                                                    let allPins = connectors @ [mount]
                                                    let structural = fixedStock @ beams
                                                    if clear mount && WalkerMotionSpace.fitsFixed [mount] (fixedStock @ occupied)
                                                       && allPins |> List.forall (fun pin ->
                                                           (structural |> List.filter (WalkerStructural.engaged pin) |> List.length)=2
                                                           && WalkerPackaging.contactCapacityDeficit pin structural 0=0) then
                                                        Some(beams,allPins)
                                                    else None))
                                    match route with
                                    | None -> possible<-false
                                    | Some(beams,connectors) -> parts.AddRange beams;pins.AddRange connectors
                            inspect |> Option.iter (fun report -> report (fun () ->
                                {Goal="frame";Rule="relocate-backbone-and-route-walking-sockets"
                                 Fixed=fixedParts |> List.map WalkerMotionSpace.partEnvelope
                                 Candidate=(moved @ List.ofSeq parts) |> List.map WalkerMotionSpace.partEnvelope
                                 Reserved=WalkerMotionSpace.envelopes space;Conflicts=[]
                                 Checked=examined;Rejected=rejected;CacheHits=hits}))
                            if possible then
                                let kept = sourceGround.Parts |> List.filter (fun p -> removed.Contains p.Id || request.Targets |> List.exists (fun t -> t.Placement.Id=p.Id))
                                yield {request.Source with
                                           Bodies=request.Source.Bodies |> List.map (fun b ->
                                               if b.Body=ground then {b with Parts=kept @ fixedParts @ moved @ List.ofSeq parts} else b)
                                           Mounts=(request.Source.Mounts |> List.filter (fun p -> removed.Contains p.Id)) @ mounts @ List.ofSeq pins}
            }
            let candidates = alternatives |> Seq.distinct |> Seq.truncate (maximumAlternatives-1) |> Seq.toList
            if request.Targets |> List.forall (fun t -> t.ToleranceLdu=Vec3.zero) then
                request.Source::candidates
            else candidates @ [request.Source])

    let options request screened = optionsObserved None request screened
