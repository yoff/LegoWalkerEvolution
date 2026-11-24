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
