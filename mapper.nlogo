;; There are two main problems that the robot must solve
;; 1) Figure out the layout of the room -- that is, what patches are solid, what aren't
;; 2) Calibrate itself the sensor range and turn speed, as well as allow for imprecise movements
;; This algorithm attempts to solve both problems at once.
;; It works by having many potential robots, each with somewhat characteristics and position.
;; Each potential robot checks it's accuracy against the sensor data. The confidence in each
;; potential robot is adjusted accordingly.
;; Then, each robot basically votes on which patches are solid and which are vacant, based on the
;; sensor data and the potential robot's attributes.
;; The best potential robots are then selected and the rest are removed.
;; The best robots reproduce with small variations.
;; The real robot and potential robots then move in an ideally similar fashion and the cycle repeats.
;;
;; Hopefully the robots conception of it's environment and itself converge.
;; The nice part of this algorithm is that it makes no assumptions about the sensitivity of the robot's
;; sensors, power of its motors, or the environment. It also requires no deduction, complex math,
;; or anything.
;;
;; It is loosely based on the particle filter algorithm used to orient robots in a known environment
;; while allowing for error in movements. This algorithm doesn't use any actual probability theory though.

extensions [ gogo ]

globals [
  serial-port
  test?
  controlable-srange
  ;; If the sensor reads > max-dist, we say it didn't hit anything
  time-step       ;; how long actions are done for
  max-confidence  ;; A patch can't have solidity > max-confidence; a robot can have confidence > max-confidence, but it only helps in selection, not reproduction
  previous-hit?
]

breed [ robots robot ]
breed [ fauxgos fauxgo ]  ;; fake gogo board robot for testing
breed [ echoes echo ]

robots-own [
  sensor-range ;; The sensors range in units of the distance the robot travels in time-step
  confidence    ;; Represents how confident we are in the bot
  turn-speed  ;; degrees the robot thinks it moves at a time
]

fauxgos-own [
  sensor-range
  turn-speed
]

echoes-own [
  sensor-range
  parent
  hit-patch
]

patches-own [
  solidity      ;; Represents how likely it is that the patch is solid
  solid?        ;; used for testing
]


to set-auton
  set autonomous-mode? true
end

to set-auton-off
  set autonomous-mode? false
end

to setup
  if not gogo:open? [ setup-gogo ]
  setup-brain
  set test? false
end

to setup-gogo
  ifelse length (gogo:ports) > 0
    [ set serial-port user-one-of "Select a port:" gogo:ports ]
    [ user-message "There is a problem with the connection. Check if the board is on, and if the cable is connected. Otherwise, try to quit NetLogo, power cycle the GoGo Board, and open NetLogo again. For more information on how to fix connection issues, refer to the NetLogo documentation or the info tab of this model"
      stop ]
  gogo:open serial-port
  repeat 5
    [ if not gogo:ping
      [ user-message "There is a problem with the connection. Check if the board is on, and if the cable is connected. Otherwise, try to quit NetLogo, power cycle the GoGo Board, and open NetLogo again. For more information on how to fix connection issues, refer to the NetLogo documentation or the info tab of this model"] ]
  gogo:talk-to-output-ports [ "a" "b" "c" "d"]
end

to setup-test
  setup-brain
  create-fauxgos 1 [
    set heading 0
    set turn-speed 5
    set shape "turtle"
    set size 3
    set sensor-range 10
  ]
  ask patches [
    set solid? pxcor = min-pxcor or
               pycor = min-pycor or
               pxcor = max-pxcor or
               pycor = max-pycor or
               (pxcor = 25 and pycor > -1) or
               (pxcor > -1 and pycor = 25) or
               (pxcor < 0 and pycor < 0 and (25 > abs (pxcor * pxcor + pycor * pycor - 625) ))
  ]
  ask patches with [solid?] [set pcolor white]
  set test? true
end

to setup-brain
  ca
  set time-step 0.05
  set max-confidence 1000
  set previous-hit? false
  create-robots 1 [
    set heading 0
    set size 5
    set confidence max-confidence
    set turn-speed init-turn-speed
    set sensor-range init-sensor-range
  ]
end

to go
  sensor-check
  select-robots
  ask robots [reproduce]
  if autonomous-mode? = true
  [autonomous-move-behavior]
end

to autonomous-move-behavior
  ifelse sense-dist > max-dist [robots-fd set previous-hit? false] [robots-rt robots-bk set previous-hit? true ]
end

to autonomous-move-behavior1
  ifelse previous-hit? = false
  [ifelse sense-dist > max-dist [robots-fd set previous-hit? false] [robots-rt robots-bk set previous-hit? true ]]
  [ifelse sense-dist > max-dist [repeat 10 [robots-bk robots-lt sensor-check] set previous-hit? false] [robots-rt robots-bk set previous-hit? true ]]   
end
;; observer procedure
;; Adjusts robots' confidences and patches' solidities based on sensor reading.
to sensor-check
  ask robots [fire-echo]
  ask robots [retract-echo]
  ask patches [
    if solidity < 0 [set solidity 0]
    if solidity > max-confidence [set solidity max-confidence]
    patch-recolor
  ]
  cd
end

;; This method will be replaced by whatever scheme we use to send the distance measure via the antennas
to-report sense-dist
  ifelse test? [
    let dist 0
    ask fauxgos [
      hatch-echoes 1 [
        set parent myself
        while [distance myself < sensor-range and not [solid?] of patch-here and can-move? echo-speed] [
          fd echo-speed
        ]
        set dist distance myself
        die
      ]
    ]
    report max-dist * (random-normal 0 1 + dist) / [sensor-range] of one-of fauxgos
  ] [report gogo:sensor 1]
end

;; robot procedure
;; Gets the sensor's output in patches based on what this robot think's the sensor's range is
to-report get-dist
  report ifelse-value learn-sensor-range? [sensor-range][init-sensor-range] * sense-dist / max-dist
end

;; robot procedure
;; Shoots an echo to the spot where the sensor sensed something (according to the robot)
;; The robots confidence level is adjusted on the way:
;; - The echo passing over a patch that the robot thinks is solid penalizes the robot
;; - The echo passing over a patch that the robot thinks is vacant rewards the robot slightly
;; - At the end, if the robot correctly predicted whether or not an obstacle was there, the robot is rewarded
;; - At the end, if the robot incorrectly predicted, the robot is penalized
;; All penalties/rewards are adjusted based on the solidity of the patches (as solidity sort of corresponds to confidence)
to fire-echo
  let dist get-dist
  hatch-echoes 1 [
    set size 1
    pd
    set parent myself
  ]
  ask echoes with [parent = myself] [
    while [distance myself < dist and can-move? echo-speed] [
      let cur-patch patch-here
      ask parent [set confidence confidence + vacant-patch-reward-factor * (5 - [solidity] of cur-patch) / 10]
      fd echo-speed
    ]
    ifelse dist < sensor-range [
      set hit-patch patch-here
      ask parent [set confidence confidence + obstacle-reward-factor * ([solidity] of [hit-patch] of myself - (max-confidence / 3)) / 10]
    ] [
      set hit-patch nobody
    ]
  ]
end

;; robot procedure
;; Like fire-echo, but adjusts solidity of the patches as the echo travels back to the robot.
;; The robots are essentially voting on what patches they think are solid
to retract-echo
  ask echoes with [parent = myself] [
    if hit-patch != nobody [ask hit-patch [ set solidity solidity + 5 ]]
    while [distance parent > 1] [
      bk echo-speed
      if patch-here != hit-patch [ ask patch-here [ set solidity solidity - 5] ]
    ]
    die
  ]
end

;; robot procedure
;; Each echo travels the same number of steps, otherwise it's biased to bigger sensor-ranges
to-report echo-speed
  report [sensor-range] of parent / 20
end

;; observer procedure
;; Natural selection - best keep-robots robots are kept, rest die
to select-robots
  if count robots > keep-robots [
    ask min-n-of (count robots - keep-robots) robots [confidence] [die]
  ]
end

;; robot procedure
;; Robots reproduce based on their confidence
to reproduce
  hatch-robots reproduce-amount [
    rt random-normal 0 .1
    if learn-sensor-range? [
      set sensor-range sensor-range + random-normal 0 sensor-range-mut-rate
    ]
    set color 5 + 10 * random 14
    if learn-turn-speed? [
      set turn-speed turn-speed + random-normal 0 turn-speed-mut-rate
    ]
  ]
end

to robots-fd
  ask robots [fd random-normal 1 vary-move]
  ifelse test? [
    ask fauxgos [
      let dist random-normal 1 0.1
      fd dist
      if [solid?] of patch-here [bk dist]
    ]
  ] [
    gogo:talk-to-output-ports ["a" "b"]
    gogo:set-output-port-power 7
    gogo:output-port-on
    wait time-step
    gogo:output-port-off
  ]
end

to robots-bk
  ask robots [bk random-normal 1 vary-move]
  ifelse test? [
    ask fauxgos [
      let dist random-normal 1 0.1
      bk dist
      if [solid?] of patch-here [fd dist]
    ]
  ] [
    gogo:talk-to-output-ports ["a" "b"]
    gogo:set-output-port-power 7
    gogo:output-port-reverse
    gogo:output-port-on
    wait time-step
    gogo:output-port-off
    gogo:output-port-reverse
  ]
end

to robots-rt
  ask robots [
    rt random-normal (ifelse-value learn-turn-speed? [turn-speed] [init-turn-speed]) vary-turn
  ]
  gogo-rt
end

to gogo-rt
  ifelse test? [
    ask fauxgos [rt turn-speed]
  ] [
    gogo:talk-to-output-ports ["a" "b"]
    gogo:set-output-port-power 7
    gogo:output-port-on
    gogo:talk-to-output-ports ["a"]
    gogo:output-port-reverse
    wait time-step
    gogo:talk-to-output-ports ["b"]
    gogo:output-port-off
    gogo:talk-to-output-ports ["a"]
    gogo:output-port-off
    gogo:output-port-reverse
  ]
end

to robots-lt
  ask robots [
    lt random-normal (ifelse-value learn-turn-speed? [turn-speed] [init-turn-speed]) vary-turn
  ]
  gogo-lt
end

to gogo-lt
  ifelse test? [
    ask fauxgos [lt turn-speed]
  ][
    gogo:talk-to-output-ports ["a" "b"]
    gogo:set-output-port-power 7
    gogo:output-port-on
    gogo:talk-to-output-ports ["b"]
    gogo:output-port-reverse
    wait time-step
    gogo:talk-to-output-ports ["a"]
    gogo:output-port-off
    gogo:talk-to-output-ports ["b"]
    gogo:output-port-off
    gogo:output-port-reverse
  ]
end

to patch-recolor
  set pcolor scale-color red solidity 0 max-confidence
end

to robot-recolor
  set color scale-color color confidence 0 max-confidence
end
@#$#@#$#@
GRAPHICS-WINDOW
481
21
996
557
50
50
5.0
1
10
1
1
1
0
1
1
1
-50
50
-50
50
0
0
1
ticks
30.0

BUTTON
13
10
79
43
NIL
setup
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

MONITOR
371
51
474
96
NIL
sense-dist
17
1
11

SLIDER
14
56
186
89
keep-robots
keep-robots
0
100
10
1
1
NIL
HORIZONTAL

SLIDER
190
56
362
89
reproduce-amount
reproduce-amount
0
100
5
1
1
NIL
HORIZONTAL

BUTTON
192
10
255
43
NIL
go
T
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

SLIDER
15
110
212
143
vacant-patch-reward-factor
vacant-patch-reward-factor
0
100
5
1
1
NIL
HORIZONTAL

SLIDER
14
149
212
182
obstacle-reward-factor
obstacle-reward-factor
0
100
10
1
1
NIL
HORIZONTAL

BUTTON
85
10
181
43
NIL
setup-test
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

SLIDER
263
107
466
140
max-dist
max-dist
0
1023
937
1
1
NIL
HORIZONTAL

BUTTON
132
429
219
462
Forward
robots-fd
NIL
1
T
OBSERVER
NIL
W
NIL
NIL
1

BUTTON
128
486
223
519
Back
robots-bk
NIL
1
T
OBSERVER
NIL
S
NIL
NIL
1

BUTTON
33
456
125
489
Turn Left
robots-lt
NIL
1
T
OBSERVER
NIL
A
NIL
NIL
1

BUTTON
225
453
326
486
Turn Right
robots-rt
NIL
1
T
OBSERVER
NIL
D
NIL
NIL
1

SWITCH
268
10
455
43
autonomous-mode?
autonomous-mode?
1
1
-1000

SLIDER
14
200
213
233
init-turn-speed
init-turn-speed
0
10
5
.1
1
NIL
HORIZONTAL

SLIDER
15
238
213
271
turn-speed-mut-rate
turn-speed-mut-rate
0
1
1
.01
1
NIL
HORIZONTAL

SWITCH
218
216
392
249
learn-turn-speed?
learn-turn-speed?
0
1
-1000

SLIDER
16
276
214
309
init-sensor-range
init-sensor-range
0
15
10
1
1
NIL
HORIZONTAL

SLIDER
16
313
214
346
sensor-range-mut-rate
sensor-range-mut-rate
0
1
1
.01
1
NIL
HORIZONTAL

SWITCH
220
288
410
321
learn-sensor-range?
learn-sensor-range?
0
1
-1000

SLIDER
16
357
188
390
vary-move
vary-move
0
1
0.2
.1
1
NIL
HORIZONTAL

SLIDER
232
357
404
390
vary-turn
vary-turn
0
5
1
.1
1
NIL
HORIZONTAL

@#$#@#$#@
## WHAT IS IT?

(a general understanding of what the model is trying to show or explain)

## HOW IT WORKS

(what rules the agents use to create the overall behavior of the model)

## HOW TO USE IT

(how to use the model, including a description of each of the items in the Interface tab)

## THINGS TO NOTICE

(suggested things for the user to notice while running the model)

## THINGS TO TRY

(suggested things for the user to try to do (move sliders, switches, etc.) with the model)

## EXTENDING THE MODEL

(suggested things to add or change in the Code tab to make the model more complicated, detailed, accurate, etc.)

## NETLOGO FEATURES

(interesting or unusual features of NetLogo that the model uses, particularly in the Code tab; or where workarounds were needed for missing features)

## RELATED MODELS

(models in the NetLogo Models Library and elsewhere which are of related interest)

## CREDITS AND REFERENCES

(a reference to the model's URL on the web if it has one, as well as any other necessary credits, citations, and links)
@#$#@#$#@
default
true
0
Polygon -7500403 true true 150 5 40 250 150 205 260 250

airplane
true
0
Polygon -7500403 true true 150 0 135 15 120 60 120 105 15 165 15 195 120 180 135 240 105 270 120 285 150 270 180 285 210 270 165 240 180 180 285 195 285 165 180 105 180 60 165 15

arrow
true
0
Polygon -7500403 true true 150 0 0 150 105 150 105 293 195 293 195 150 300 150

box
false
0
Polygon -7500403 true true 150 285 285 225 285 75 150 135
Polygon -7500403 true true 150 135 15 75 150 15 285 75
Polygon -7500403 true true 15 75 15 225 150 285 150 135
Line -16777216 false 150 285 150 135
Line -16777216 false 150 135 15 75
Line -16777216 false 150 135 285 75

bug
true
0
Circle -7500403 true true 96 182 108
Circle -7500403 true true 110 127 80
Circle -7500403 true true 110 75 80
Line -7500403 true 150 100 80 30
Line -7500403 true 150 100 220 30

butterfly
true
0
Polygon -7500403 true true 150 165 209 199 225 225 225 255 195 270 165 255 150 240
Polygon -7500403 true true 150 165 89 198 75 225 75 255 105 270 135 255 150 240
Polygon -7500403 true true 139 148 100 105 55 90 25 90 10 105 10 135 25 180 40 195 85 194 139 163
Polygon -7500403 true true 162 150 200 105 245 90 275 90 290 105 290 135 275 180 260 195 215 195 162 165
Polygon -16777216 true false 150 255 135 225 120 150 135 120 150 105 165 120 180 150 165 225
Circle -16777216 true false 135 90 30
Line -16777216 false 150 105 195 60
Line -16777216 false 150 105 105 60

car
false
0
Polygon -7500403 true true 300 180 279 164 261 144 240 135 226 132 213 106 203 84 185 63 159 50 135 50 75 60 0 150 0 165 0 225 300 225 300 180
Circle -16777216 true false 180 180 90
Circle -16777216 true false 30 180 90
Polygon -16777216 true false 162 80 132 78 134 135 209 135 194 105 189 96 180 89
Circle -7500403 true true 47 195 58
Circle -7500403 true true 195 195 58

circle
false
0
Circle -7500403 true true 0 0 300

circle 2
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240

cow
false
0
Polygon -7500403 true true 200 193 197 249 179 249 177 196 166 187 140 189 93 191 78 179 72 211 49 209 48 181 37 149 25 120 25 89 45 72 103 84 179 75 198 76 252 64 272 81 293 103 285 121 255 121 242 118 224 167
Polygon -7500403 true true 73 210 86 251 62 249 48 208
Polygon -7500403 true true 25 114 16 195 9 204 23 213 25 200 39 123

cylinder
false
0
Circle -7500403 true true 0 0 300

dot
false
0
Circle -7500403 true true 90 90 120

face happy
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 255 90 239 62 213 47 191 67 179 90 203 109 218 150 225 192 218 210 203 227 181 251 194 236 217 212 240

face neutral
false
0
Circle -7500403 true true 8 7 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Rectangle -16777216 true false 60 195 240 225

face sad
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 168 90 184 62 210 47 232 67 244 90 220 109 205 150 198 192 205 210 220 227 242 251 229 236 206 212 183

fish
false
0
Polygon -1 true false 44 131 21 87 15 86 0 120 15 150 0 180 13 214 20 212 45 166
Polygon -1 true false 135 195 119 235 95 218 76 210 46 204 60 165
Polygon -1 true false 75 45 83 77 71 103 86 114 166 78 135 60
Polygon -7500403 true true 30 136 151 77 226 81 280 119 292 146 292 160 287 170 270 195 195 210 151 212 30 166
Circle -16777216 true false 215 106 30

flag
false
0
Rectangle -7500403 true true 60 15 75 300
Polygon -7500403 true true 90 150 270 90 90 30
Line -7500403 true 75 135 90 135
Line -7500403 true 75 45 90 45

flower
false
0
Polygon -10899396 true false 135 120 165 165 180 210 180 240 150 300 165 300 195 240 195 195 165 135
Circle -7500403 true true 85 132 38
Circle -7500403 true true 130 147 38
Circle -7500403 true true 192 85 38
Circle -7500403 true true 85 40 38
Circle -7500403 true true 177 40 38
Circle -7500403 true true 177 132 38
Circle -7500403 true true 70 85 38
Circle -7500403 true true 130 25 38
Circle -7500403 true true 96 51 108
Circle -16777216 true false 113 68 74
Polygon -10899396 true false 189 233 219 188 249 173 279 188 234 218
Polygon -10899396 true false 180 255 150 210 105 210 75 240 135 240

house
false
0
Rectangle -7500403 true true 45 120 255 285
Rectangle -16777216 true false 120 210 180 285
Polygon -7500403 true true 15 120 150 15 285 120
Line -16777216 false 30 120 270 120

leaf
false
0
Polygon -7500403 true true 150 210 135 195 120 210 60 210 30 195 60 180 60 165 15 135 30 120 15 105 40 104 45 90 60 90 90 105 105 120 120 120 105 60 120 60 135 30 150 15 165 30 180 60 195 60 180 120 195 120 210 105 240 90 255 90 263 104 285 105 270 120 285 135 240 165 240 180 270 195 240 210 180 210 165 195
Polygon -7500403 true true 135 195 135 240 120 255 105 255 105 285 135 285 165 240 165 195

line
true
0
Line -7500403 true 150 0 150 300

line half
true
0
Line -7500403 true 150 0 150 150

pentagon
false
0
Polygon -7500403 true true 150 15 15 120 60 285 240 285 285 120

person
false
0
Circle -7500403 true true 110 5 80
Polygon -7500403 true true 105 90 120 195 90 285 105 300 135 300 150 225 165 300 195 300 210 285 180 195 195 90
Rectangle -7500403 true true 127 79 172 94
Polygon -7500403 true true 195 90 240 150 225 180 165 105
Polygon -7500403 true true 105 90 60 150 75 180 135 105

plant
false
0
Rectangle -7500403 true true 135 90 165 300
Polygon -7500403 true true 135 255 90 210 45 195 75 255 135 285
Polygon -7500403 true true 165 255 210 210 255 195 225 255 165 285
Polygon -7500403 true true 135 180 90 135 45 120 75 180 135 210
Polygon -7500403 true true 165 180 165 210 225 180 255 120 210 135
Polygon -7500403 true true 135 105 90 60 45 45 75 105 135 135
Polygon -7500403 true true 165 105 165 135 225 105 255 45 210 60
Polygon -7500403 true true 135 90 120 45 150 15 180 45 165 90

sheep
false
15
Circle -1 true true 203 65 88
Circle -1 true true 70 65 162
Circle -1 true true 150 105 120
Polygon -7500403 true false 218 120 240 165 255 165 278 120
Circle -7500403 true false 214 72 67
Rectangle -1 true true 164 223 179 298
Polygon -1 true true 45 285 30 285 30 240 15 195 45 210
Circle -1 true true 3 83 150
Rectangle -1 true true 65 221 80 296
Polygon -1 true true 195 285 210 285 210 240 240 210 195 210
Polygon -7500403 true false 276 85 285 105 302 99 294 83
Polygon -7500403 true false 219 85 210 105 193 99 201 83

square
false
0
Rectangle -7500403 true true 30 30 270 270

square 2
false
0
Rectangle -7500403 true true 30 30 270 270
Rectangle -16777216 true false 60 60 240 240

star
false
0
Polygon -7500403 true true 151 1 185 108 298 108 207 175 242 282 151 216 59 282 94 175 3 108 116 108

target
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240
Circle -7500403 true true 60 60 180
Circle -16777216 true false 90 90 120
Circle -7500403 true true 120 120 60

tree
false
0
Circle -7500403 true true 118 3 94
Rectangle -6459832 true false 120 195 180 300
Circle -7500403 true true 65 21 108
Circle -7500403 true true 116 41 127
Circle -7500403 true true 45 90 120
Circle -7500403 true true 104 74 152

triangle
false
0
Polygon -7500403 true true 150 30 15 255 285 255

triangle 2
false
0
Polygon -7500403 true true 150 30 15 255 285 255
Polygon -16777216 true false 151 99 225 223 75 224

truck
false
0
Rectangle -7500403 true true 4 45 195 187
Polygon -7500403 true true 296 193 296 150 259 134 244 104 208 104 207 194
Rectangle -1 true false 195 60 195 105
Polygon -16777216 true false 238 112 252 141 219 141 218 112
Circle -16777216 true false 234 174 42
Rectangle -7500403 true true 181 185 214 194
Circle -16777216 true false 144 174 42
Circle -16777216 true false 24 174 42
Circle -7500403 false true 24 174 42
Circle -7500403 false true 144 174 42
Circle -7500403 false true 234 174 42

turtle
true
0
Polygon -10899396 true false 215 204 240 233 246 254 228 266 215 252 193 210
Polygon -10899396 true false 195 90 225 75 245 75 260 89 269 108 261 124 240 105 225 105 210 105
Polygon -10899396 true false 105 90 75 75 55 75 40 89 31 108 39 124 60 105 75 105 90 105
Polygon -10899396 true false 132 85 134 64 107 51 108 17 150 2 192 18 192 52 169 65 172 87
Polygon -10899396 true false 85 204 60 233 54 254 72 266 85 252 107 210
Polygon -7500403 true true 119 75 179 75 209 101 224 135 220 225 175 261 128 261 81 224 74 135 88 99

wheel
false
0
Circle -7500403 true true 3 3 294
Circle -16777216 true false 30 30 240
Line -7500403 true 150 285 150 15
Line -7500403 true 15 150 285 150
Circle -7500403 true true 120 120 60
Line -7500403 true 216 40 79 269
Line -7500403 true 40 84 269 221
Line -7500403 true 40 216 269 79
Line -7500403 true 84 40 221 269

wolf
false
0
Polygon -16777216 true false 253 133 245 131 245 133
Polygon -7500403 true true 2 194 13 197 30 191 38 193 38 205 20 226 20 257 27 265 38 266 40 260 31 253 31 230 60 206 68 198 75 209 66 228 65 243 82 261 84 268 100 267 103 261 77 239 79 231 100 207 98 196 119 201 143 202 160 195 166 210 172 213 173 238 167 251 160 248 154 265 169 264 178 247 186 240 198 260 200 271 217 271 219 262 207 258 195 230 192 198 210 184 227 164 242 144 259 145 284 151 277 141 293 140 299 134 297 127 273 119 270 105
Polygon -7500403 true true -1 195 14 180 36 166 40 153 53 140 82 131 134 133 159 126 188 115 227 108 236 102 238 98 268 86 269 92 281 87 269 103 269 113

x
false
0
Polygon -7500403 true true 270 75 225 30 30 225 75 270
Polygon -7500403 true true 30 75 75 30 270 225 225 270

@#$#@#$#@
NetLogo 5.0.3
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
default
0.0
-0.2 0 1.0 0.0
0.0 1 1.0 0.0
0.2 0 1.0 0.0
link direction
true
0
Line -7500403 true 150 150 90 180
Line -7500403 true 150 150 210 180

@#$#@#$#@
0
@#$#@#$#@
