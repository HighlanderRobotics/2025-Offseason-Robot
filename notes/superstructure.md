# Superstructure Graph

```mermaid
%%{init: {"flowchart": {"defaultRenderer": "elk"}} }%%
graph TD;

IDLE

subgraph CORAL
subgraph INTAKE_CORAL

IDLE --> PRE_INTAKE_GROUND
PRE_INTAKE_GROUND --> INTAKE_CORAL_GROUND
INTAKE_CORAL_GROUND --> POST_INTAKE_CORAL_GROUND
POST_INTAKE_CORAL_GROUND --> IDLE

end

subgraph SCORE_CORAL

IDLE --- L1
IDLE --> PRE_L2
PRE_L2 --> L2
L2 --> POST_L2
POST_L2 --> IDLE

IDLE --> PRE_L3
PRE_L3 --> L3
L3 --> POST_L3
POST_L3 --> IDLE

IDLE --> PRE_L4
PRE_L4 --> L4
L4 --> POST_L4
POST_L4 --> IDLE

end

end

subgraph ALGAE

subgraph INTAKE_ALGAE

IDLE --- INTAKE_ALGAE_REEF_HIGH
IDLE --- INTAKE_ALGAE_REEF_LOW
IDLE --- INTAKE_ALGAE_GROUND

end

subgraph SCORE_ALGAE

IDLE --- BARGE
IDLE --- PROCESSOR

end

end

subgraph climb [CLIMB]
IDLE --> PRE_CLIMB
PRE_CLIMB --> CLIMB
end
