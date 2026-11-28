namespace LegoWalkers

open System.Text.Json.Serialization

type WalkerPlacement = { Id: string; Part: string; Color: int; Position: Vec3; Rotation: Rotation }
type WalkerBodyChoice = { Body: string; Parts: WalkerPlacement list }
type WalkerPivotChoice = { Group: string; Hinges: string list; Placement: WalkerPlacement }
type WalkerFootContact = { Part: string; Depth: float }
type WalkerFootChoice = {
    Foot: string
    Part: string
    Depth: float
    [<JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)>]
    AdditionalContacts: WalkerFootContact list option
}

module WalkerFootChoice =
    let contacts (foot: WalkerFootChoice) =
        {Part=foot.Part;Depth=foot.Depth} :: defaultArg foot.AdditionalContacts []

    let shiftParts ids delta (foot: WalkerFootChoice) =
        {foot with Depth=foot.Depth+(if Set.contains foot.Part ids then delta else 0.0)
                   AdditionalContacts=foot.AdditionalContacts |> Option.map (List.map (fun contact ->
                       {contact with Depth=contact.Depth+(if Set.contains contact.Part ids then delta else 0.0)}))}

type LegBodyBinding = { Role: string; Body: string }
type LegAttachment = { Role: string; Port: MotionPort }
type LegInstance = {
    Id: string
    Foot: string
    Mirrored: bool
    Phase: float
    Bodies: LegBodyBinding list
    Attachments: LegAttachment list
}
type LegFamily = { Id: string; Instances: LegInstance list }
type WalkerGenome = {
    SchemaVersion: int
    RecipeVersion: string
    MechanismId: string
    Remote: bool
    TowWagon: bool
    Bodies: WalkerBodyChoice list
    Connections: WalkerPivotChoice list
    Mounts: WalkerPlacement list
    Feet: WalkerFootChoice list
    MotorPart: string
    Speed: float
    MaxTorque: float
    [<JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)>]
    LegFamilies: LegFamily list option
}

type WalkerFootStack = { ContactPart: string; PartIds: string list }

module WalkerFootStack =
    let tryCreate (parts: PartInstance list) (contact: PartInstance) =
        let rec mounted layers (plate: PartInstance) =
            if layers>2 || plate.Part<>"3710.dat" || plate.Body.IsNone
               || not(Vec3.finite plate.Position && Rotation.valid plate.Rotation) then None else
            parts |> List.sortBy _.Id |> List.tryPick (fun parent ->
                if parent.Id=plate.Id || parent.Body<>plate.Body
                   || Vec3.distance parent.Rotation.X plate.Rotation.X>=1e-6
                   || Vec3.distance parent.Rotation.Y plate.Rotation.Y>=1e-6
                   || Vec3.distance parent.Rotation.Z plate.Rotation.Z>=1e-6
                   || not(Vec3.finite parent.Position && Rotation.valid parent.Rotation) then None else
                match Catalog.soleUnderside parent.Part with
                | Some height when Vec3.distance
                                       (Vec3.add parent.Position (Vec3.scale height parent.Rotation.Y))
                                       plate.Position<1e-6 ->
                    if parent.Part="3701.dat" then Some [plate.Id;parent.Id]
                    elif parent.Part="3710.dat" then
                        mounted (layers+1) parent |> Option.map (fun ids -> plate.Id::ids)
                    else None
                | _ -> None)
        mounted 1 contact |> Option.map (fun ids -> {ContactPart=contact.Id;PartIds=ids})
