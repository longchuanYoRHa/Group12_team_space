# Group12-team-space

## AERO62520 Coursework Schedule

```mermaid
gantt
    title AERO62520 Group 12 - Coursework Schedule
    dateFormat  YYYY-MM-DD
    axisFormat  %b %d
    excludes    weekends

    section CW1 - Workplace Charter
    Charter Development    : cw1, 2025-10-06, 9d
    CW1 Submission        : milestone, cw1_due, 2025-10-17, 0d

    section CW2 - Initial CPD Portfolio
    CPD Portfolio Work     : cw2, 2025-10-17, 10d
    CW2 Submission        : milestone, cw2_due, 2025-10-31, 0d

    section CW3 - Design Requirement Analysis
    Requirements Analysis  : crit, cw3, 2025-10-31, 10d
    CW3 Submission        : milestone, cw3_due, 2025-11-14, 0d

    section CW4 - Preliminary Design Review
    Design Development     : crit, cw4, 2025-11-14, 20d
    CW4 Submission        : milestone, cw4_due, 2025-12-12, 0d

    section CW5 - Technical Competency Assessment
    Technical Assessment   : crit, cw5, 2025-11-14, 20d
    CW5 Submission        : milestone, cw5_due, 2025-12-12, 0d
```

## General goal of the project

Using the given Leo Rover and the robotic arm, build a rover that can navigate through maze and able to pick up the objects in the maze.

## Git workspace structure

- [Mechanical_design](Mechanical_design): Mechanical design of the rover and the robotic arm , with some general draft and the design requirements.
- [Software_control](Software_control): Software control of the rover and the robotic arm, with some general glance and the software control requirements.
- [CW1_workplace_charter](CW1_workplace_charter)

## Recent Tasks

**Chassis assembly** 

Problems: 

- the suspention is still loose
- the baring of one wheel is not tight enough

solved：
- missing components for connecting the wheel structure to the chassis.
- wrong port on the given Powerbox

**Remote connection with the rover(done)** 

**ROS example run(done)**

**Joystick control**

problems:

- joystick mapping yaml file is not working
(changing the first line of the yaml file to the correct mapping)

**sensor test(done)**

**NUC setup**

**robot arm test**