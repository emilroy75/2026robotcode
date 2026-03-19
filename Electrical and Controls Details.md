# CAN Devices

| CAN ID | Abreviation | Name                | Device Type            | Motion Notes             |
| -------- | ------------- | --------------------- | ------------------------ | -------------------------- |
| 1      | FLD         | Front Left Drive    | Spark Flex             | Spins                    |
| 2      | FLT         | Front Left Turn     | Spark Max              | Positive = Spins CCW     |
| 3      | FRD         | Front Right Drive   | Spark Flex             | Spins                    |
| 4      | FRT         | Front Right Turn    | Spark Max              | Positive = Spins CCW     |
| 5      | BLD         | Back Left Drive     | Spark Flex             | Spins                    |
| 6      | BLT         | Back Left Turn      | Spark Max              | Positive = Spins CCW     |
| 7      | BRD         | Back Right Drive    | Spark Flex             | Spins                    |
| 8      | BRT         | Back Right Turn     | Spark Max              | Positive = Spins CCW     |
| 9      | INR         | Intake Rollers      | Spark Max              | Positive = Intake Ball   |
| 10     | IND         | Intake Deploy       | Spark Max              | Positive = Retract       |
| 11     | SEF         | Shooter Eject Front | Spark Max              | Forward = Shoot          |
| 12     | SEB         | Shooter Eject Back  | Spark Max              | Reverse = Shoot          |
| 13     | SFD         | Shooter Feeder      | Spark Max              | Positive = Reject Ball   |
| 14     | CLM         | Climber             | Spark Max              | TBD                      |
| 15     | AGT         | Agitator            | Spark Max              | Positive = Into Shooter  |
| 0      | n/a         | RIO                 | RoboRIO                |                          |
| 0      | n/a         | PDH                 | Power Distrabution Hub |                          |

# CAN Topology

<<<<<<< HEAD
(Term) RIO -> SEB -> BRD -> BRT -> CLM-> SFD -> FRT -> SEF -> FRD -> FLT -> INR -> IND -> FLD -> BLT -> BLD-> PDH -> AGI (Term)
=======
(Term) RIO -> SEB -> BRD -> BRT -> CLM-> SFD -> FRT -> SEF -> FRD -> FLT -> INR -> IND -> FLD -> BLT -> BLD-> PDH -> AGI (Term)
>>>>>>> 9a7cbe370edca70db91c8e4f4b07b83d58388bce

# PDH Connections


| Slot | Breaker/Fuse (Amps) | Load           |
| ------ | --------------------- | --------------------------- |
| 0    | 40                  | BLD                       |
| 1    | 20                  | BLT                       |
| 2    | 40                  | FLD                       |
| 3    | 20                  | FLT                       |
| 4    | 30                  | INR                       |
| 5    | 30                  | IND                       |
| 6    | -                   | -                         |
| 7    | -                   | -                         |
| 8    | -                   | -                         |
| 9    | 30                  | AGI                       |
| 10   | -                   | -                         |
| 11   | -                   | -                         |
| 12   | 30                  | SFD                       |
| 13   | 20                  | FRT                       |
| 14   | 40                  | SEF                       |
| 15   | 40                  | SEB                       |
| 16   | 40                  | FRD                       |
| 17   | 40                  | BRD                       |
| 18   | 40                  | CLM                       |
| 19   | 20                  | BRT                       |
| 20   | 10                  | Raspberry PI Power Supply |
| 21   | 10                  | RoboRio                   |
| 22   | 10                  | Radio                     |
| 23   | -                   | (Empty)                   |


# IP Addresses

<<<<<<< HEAD
10.2.50.1 - Radio
10.2.50.2 - RoboRIO
10.2.50.6 - RaspberryPi (Photon Vision)
10.2.50.7 - RaspberryPi (Machine Vision)
=======
10.2.50.1 - Radio
10.2.50.2 - RoboRIO
10.2.50.6 - RaspberryPi (Photon Vision)
10.2.50.7 - RaspberryPi (Machine Vision)
>>>>>>> 9a7cbe370edca70db91c8e4f4b07b83d58388bce
