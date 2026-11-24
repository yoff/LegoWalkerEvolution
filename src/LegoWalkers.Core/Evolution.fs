namespace LegoWalkers

open System

module Evolution =
    let private differences (a: Genome) (b: Genome) = [
        if a.Pairs <> b.Pairs then "pairs"
        if a.Remote <> b.Remote then "remote"
        if a.TowWagon <> b.TowWagon then "towWagon"
        if a.BatteryOnWagon <> b.BatteryOnWagon then "batteryOnWagon"
        if a.ReceiverOnWagon <> b.ReceiverOnWagon then "receiverOnWagon"
        if a.WagonAxleHole <> b.WagonAxleHole then "wagonAxleHole"
        if a.GroundHoles <> b.GroundHoles then "groundHoles"
        if a.CouplerHoles <> b.CouplerHoles then "couplerHoles"
        if a.RockerHoles <> b.RockerHoles then "rockerHoles"
        if a.PhaseQuarterTurns <> b.PhaseQuarterTurns then "phaseQuarterTurns"
        if a.MotorDirection <> b.MotorDirection then "motorDirection"
        if a.BatterySlot <> b.BatterySlot then "batterySlot"
        if a.BatteryLevel <> b.BatteryLevel then "batteryLevel"
        if a.ReceiverSlot <> b.ReceiverSlot then "receiverSlot"
        if a.ReceiverSide <> b.ReceiverSide then "receiverSide"
    ]

    let rec propose pairs remote towing seed attempt parent =
        if attempt < 0 then invalidArg (nameof attempt) "attempt must be nonnegative."
        let initial =
            { Genome.defaults with
                Pairs = pairs; Remote = remote; Seed = seed
                TowWagon = towing; BatteryOnWagon = remote && towing }
        match parent with
        | None when attempt = 0 -> initial, "founder", []
        | None ->
            let genome, _, changes = propose pairs remote towing seed (attempt-1) (Some initial)
            genome, "alternative-founder", changes
        | Some (source: Genome) when source.Pairs <> pairs ->
            let resized = { source with Pairs = pairs; Seed = seed }
            if source.Remote = remote && source.TowWagon = towing && attempt = 0 then
                resized, "change-foot-count", differences source resized
            else
                let nextAttempt = if source.Remote = remote && source.TowWagon = towing then attempt-1 else attempt
                let genome, operation, _ = propose pairs remote towing seed nextAttempt (Some resized)
                genome, "change-foot-count-and-" + operation, differences source genome
        | Some source when source.Remote <> remote ->
            if not remote then invalidArg (nameof parent) "Payload initialization must go from wired to remote."
            let loaded =
                { initial with GroundHoles = source.GroundHoles; CouplerHoles = source.CouplerHoles
                               RockerHoles = source.RockerHoles; PhaseQuarterTurns = source.PhaseQuarterTurns
                               MotorDirection = source.MotorDirection }
            if attempt = 0 then loaded, "add-electronics", differences source loaded
            else
                let genome, _, _ = propose pairs remote towing seed (attempt-1) (Some loaded)
                genome, "add-electronics-and-mutate", differences source genome
        | Some source when source.TowWagon <> towing ->
            if not towing then invalidArg (nameof parent) "Wagon initialization must go from solo to towing."
            let loaded = {source with Seed = seed; TowWagon = true; BatteryOnWagon = remote; ReceiverOnWagon = false}
            if attempt = 0 then loaded, "attach-wagon", differences source loaded
            else
                let genome, _, _ = propose pairs remote towing seed (attempt-1) (Some loaded)
                genome, "attach-wagon-and-mutate", differences source genome
        | Some source ->
            let g = { source with Seed = seed }
            let choices = [
                yield { g with MotorDirection = -g.MotorDirection }, "motorDirection"
                for value in [3; 4] do
                    if value <> g.GroundHoles then yield {g with GroundHoles = value}, "groundHoles"
                    if value <> g.CouplerHoles then yield {g with CouplerHoles = value}, "couplerHoles"
                    if value <> g.RockerHoles then yield {g with RockerHoles = value}, "rockerHoles"
                for value in 0..3 do
                    if value <> g.PhaseQuarterTurns then yield {g with PhaseQuarterTurns = value}, "phaseQuarterTurns"
                if remote then
                    for value in 0..2 do
                        if value <> g.BatterySlot then yield {g with BatterySlot = value}, "batterySlot"
                        if value <> g.ReceiverSlot then yield {g with ReceiverSlot = value}, "receiverSlot"
                    yield {g with BatteryLevel = 1-g.BatteryLevel}, "batteryLevel"
                    yield {g with ReceiverSide = 1-g.ReceiverSide}, "receiverSide"
                    if towing then
                        yield {g with BatteryOnWagon = not g.BatteryOnWagon}, "batteryOnWagon"
                        yield {g with ReceiverOnWagon = not g.ReceiverOnWagon}, "receiverOnWagon"
                if towing then
                    for value in 2..4 do
                        if value <> g.WagonAxleHole then yield {g with WagonAxleHole = value}, "wagonAxleHole"
            ]
            let random = Random seed
            let ordered = choices |> List.map (fun value -> random.Next(), value) |> List.sortBy fst |> List.map snd
            let candidate, field = ordered[attempt % ordered.Length]
            candidate, "single-gene-mutation", [field]
