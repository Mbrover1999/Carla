# סקירת קוד מלאה — Autonomous Driving Safety System

## 1. מטרת הפרויקט

הפרויקט מדגים רכב אוטונומי בסביבת CARLA המשלב בין מודל למידת מכונה לבין מערכות ניווט ובטיחות דטרמיניסטיות. מודל ה־AI אחראי בעיקר לחיזוי זווית ההיגוי מתוך תמונת המצלמה. החלטות שבהן נדרשת התנהגות צפויה, ניתנת להסבר ובטוחה — מהירות, צמתים, רמזורים, מכשולים, שמירת נתיב, תנועה חוצה, שטח מת ותגובה לאובדן שליטה — מטופלות באמצעות נתוני CARLA, גאומטריה, ספים ומכונות מצבים.

זוהי ארכיטקטורה היברידית:

```text
RGB Camera → Steering CNN ───────────────┐
                                         │
CARLA Map → Route + Intersection Control ├→ Requested Control
                                         │
Road Speed Limit → Speed Controller ─────┘
                                                ↓
Sensors + World State → Independent Safety Layers
                                                ↓
                                      Final Vehicle Control
                                                ↓
                                 Display + Sounds + CSV Log + Score
```

היתרון המרכזי הוא הפרדת אחריות. גם אם מודל ההיגוי טועה או מפסיק להגיב, שכבות הבטיחות אינן תלויות בו ויכולות לשנות את פקודת הרכב הסופית.

## 2. זרימת התוכנית בזמן ריצה

1. `interface.py` מציג למשתמש את מסך הפתיחה ובחירת התרחיש.
2. הממשק מפעיל את `main.py` כתהליך Python נפרד ומעביר אליו את מזהה התרחיש, זמן הריצה וכמות התנועה.
3. `main.py` מתחבר ל־CARLA, מנקה רכבים ישנים, יוצר את רכב ה־ego, מכין את התרחיש, מוסיף תנועה וחיישנים ובוחר בקר נהיגה.
4. `simulation.py` מפעיל לולאה בכל frame חדש של המצלמה.
5. הבקר יוצר פקודת steering/throttle/brake ראשונית.
6. בקר הצומת ובקר המהירות מעדכנים אותה בהתאם למסלול ולמהירות החוקית.
7. התרחיש רשאי להזריק אירוע מבוקר, למשל סטייה או חוסר פעילות.
8. שכבות הבטיחות מופעלות בזו אחר זו. כל שכבה שומרת על בלימה שכבר נדרשה על ידי שכבה קודמת.
9. הפקודה הסופית נשלחת לרכב, מוצגת בחלון, נרשמת ל־CSV ונכללת בחישוב הציון.
10. בסיום מתבצע cleanup של חיישנים, רכבים, צלילים וחלונות.

סדר עיבוד הפקודה חשוב:

```text
AI steering
→ intersection steering override
→ road-speed control
→ scenario control injection
→ forward-obstacle safety
→ cross-traffic safety
→ traffic-light safety
→ inactivity detection
→ lane-keeping correction, כאשר מותר
→ emergency pull-over
→ final apply_control
```

## 3. קובצי האתחול והתשתית

### `config.py`

מרכז את כל פרמטרי המערכת במקום אחד. במקום לפזר “מספרי קסם” בין האלגוריתמים, כל טווח, זמן תגובה, gain ומגבלת מהירות מוגדרים כאן.

הקבוצות המרכזיות הן:

- חיבור ל־CARLA: כתובת IPv4 מפורשת `127.0.0.1`, פורט `2000`, timeout ומפת `Town10HD`.
- סימולציה ורכב: משך ברירת מחדל, מספר רכבי תנועה ו־blueprint של Tesla Model 3.
- מצלמה: רזולוציה, FOV, קצב דגימה ומיקום פיזי על הרכב.
- מודל: נתיב לקובץ המשקולות ורזולוציית הקלט.
- בקר AI: scaling, smoothing ומגבלות steering/throttle/brake.
- חיישני מכשול ו־Radar.
- שכבות בטיחות: מרחקי האטה ובלימה, זמני תגובה וספי מהירות.
- Safe Stop: מהירות יעד, מרחקי clearance, הגבלת steering וזמני צפירה/שיחה.
- נתיב, רמזורים ותנועה חוצה.
- ניווט, בקר צמתים ובקר מהירות לפי תמרור.

זהו הקובץ העיקרי לכיול התנהגות הרכב ללא שינוי בלוגיקה עצמה.

### `carla_client.py`

הפונקציה `connect_to_carla()` יוצרת `carla.Client`, מגדירה timeout ומבקשת את העולם הפעיל. אם המפה הנוכחית אינה `Town10HD`, היא טוענת אותה. במקרה של timeout נזרקת שגיאה עם הסבר ברור למשתמש. השימוש ב־`127.0.0.1` מונע מצב שבו Windows פותר `localhost` ל־IPv6 בזמן שהשרת מאזין ב־IPv4.

### `vehicles.py`

`spawn_ego_vehicle()` מערבב את נקודות ה־spawn ומנסה להציב את רכב ה־ego עד שנמצאת נקודה פנויה. שימוש ב־`try_spawn_actor` מונע exception כאשר נקודה תפוסה.

`spawn_traffic_vehicles()` פועל בצורה דומה עבור רכבי התנועה, בוחר blueprints אקראיים ומפעיל עבורם CARLA Autopilot. הפונקציה מחזירה רק את הרכבים שנוצרו בפועל, כך שה־cleanup יודע בדיוק אילו actors למחוק.

### `cleanup.py`

`destroy_existing_vehicles()` מנקה רכבים שנשארו מריצה קודמת. פעולה זו חשובה במיוחד לאחר crash או עצירה לא תקינה.

`cleanup()` עוצר תחילה callbacks חדשים של החיישנים, עוצר ומשמיד כל sensor, משמיד את הרכבים שנוצרו וסוגר את חלונות OpenCV. כל actor נבדק באמצעות `is_alive`, ושגיאה בניקוי actor אחד אינה מונעת ניקוי של האחרים.

### `main.py`

זהו קובץ ה־bootstrap של הסימולציה.

- `create_control_command_reader()` מממש IPC פשוט מבוסס קובץ. הממשק מוסיף פקודות לשורות בקובץ זמני, והסימולציה קוראת בכל פעם רק את התוכן החדש באמצעות שמירת מיקום הקריאה.
- `create_controller()` בוחר בין `AIController` לבין `AutopilotController` לפי `DRIVING_MODE`.
- `main()` מבצע את סדר האתחול המלא: חיבור, ניקוי, spawn, תרחיש, traffic, חיישנים, controller, logger ולולאת הסימולציה.
- יצירת Radars עטופה ב־fallback. אם גרסת CARLA אינה תומכת בהם, שאר המערכת ממשיכה באמצעות מידע ה־actors של CARLA.
- `parse_arguments()` ו־`run_from_command_line()` מאפשרים להפעיל את אותה מערכת גם מהממשק וגם ישירות מהטרמינל.
- בלוק `finally` מבטיח cleanup גם במקרה של exception.

## 4. חיישנים ואיסוף נתונים

### `sensors.py`

הקובץ יוצר את חיישני CARLA ומתרגם callbacks אסינכרוניים למצב שהלולאה הראשית יכולה לקרוא בבטחה.

לכל סוג מידע יש state גלובלי ו־`threading.Lock`, משום ש־CARLA מפעילה callbacks של sensors מחוץ ללולאת העיבוד הראשית.

- מצלמת RGB: `process_rgb_image()` ממירה buffer מסוג BGRA למערך NumPy מסוג BGR המתאים ל־OpenCV. מספר ה־frame נשמר כדי שהמודל לא יעבד אותה תמונה פעמיים.
- Obstacle Sensor: שומר את מרחק המכשול וה־actor שפגע בקרן. קריאה ישנה ביותר מ־0.15 שניות נחשבת לא רלוונטית, כדי שהרכב לא ימשיך לבלום אחרי שהמכשול נעלם.
- Collision Sensor: שומר latch של התנגשות ואת ה־actor השני.
- Lane Invasion Sensor: ממיר את סוגי סימוני הכביש לשמות ושומר event key לפי מספר frame.
- Blind-spot Radars: שני sensors ממוקמים בחלק האחורי, בזוויות שמאלה וימינה. מכל measurement נשמרים העומק הקרוב ביותר, מהירות יחסית, מספר detections וזמן הקריאה.
- פונקציות `get_*` מחזירות העתקים של המצב תחת lock.
- פונקציות `create_*` מגדירות blueprint, transform, attributes ו־callback לכל sensor.
- `stop_sensors()` מונע מ־callbacks לעדכן state בזמן תהליך הסגירה.

### `data_collector.py`

אחראי ליצירת dataset עבור אימון מודל ההיגוי.

- התמונות נשמרות תחת `dataset/images` והתגיות ב־`driving_log.csv`.
- כל שורה כוללת image path, frame, steering, throttle, brake ומהירות.
- קצב הדגימה אינו קבוע: נשמרות יותר תמונות בפניות חדות ובינוניות ופחות בנסיעה ישרה או בעמידה. כך מצמצמים עודף דוגמאות של steering≈0.
- הקובץ נפתח במצב append, header נוצר רק בקובץ חדש, ומתבצע flush תקופתי להגנה מאובדן מידע.

## 5. מודל ההיגוי והלמידה

### `training/model.py`

מגדיר את `SteeringModel`, רשת CNN לרגרסיה של ערך steering יחיד.

חלק ה־features כולל ארבע שכבות convolution עם ReLU. מספר הערוצים גדל מ־24 עד 64 והרזולוציה יורדת באמצעות stride=2. `AdaptiveAvgPool2d(4, 8)` נותן גודל features קבוע גם אם קיימים הבדלים קטנים בגודל הקלט.

חלק ה־regressor משטח את ה־features ומעביר אותם דרך שכבות fully connected בגודל 256 ו־64. `Dropout(0.25)` מפחית overfitting. שכבת `Tanh` האחרונה מגבילה את החיזוי לטווח `[-1, 1]`.

### `training/driving_dataset.py`

`DrivingDataset` מחבר בין ה־CSV לתמונות.

- מוודא שקיימות העמודות `image_path` ו־`steering`.
- חותך את 35% העליונים של התמונה כדי להפחית שמיים ובניינים ולהתמקד בכביש.
- משנה גודל ל־180×320, ממיר ל־tensor ומנרמל לערכים סביב `[-1,1]`.
- במצב training מופעל `ColorJitter` לשינוי תאורה וצבע.
- flip אופקי הופך גם את סימן ה־steering, מפני שפנייה ימינה הופכת לפנייה שמאלה.
- `_reduce_straight_samples()` משאיר את כל דוגמאות הפנייה ורק 40% מדוגמאות הנסיעה הישרה. כך המודל אינו לומד להחזיר אפס כמעט תמיד.

### `training/train.py`

מנהל את תהליך האימון.

- קובע seed קבוע לשחזור תוצאות.
- בוחר CUDA, אחריו Apple MPS, ולבסוף CPU.
- מפצל את הנתונים לפי זמן: החלק האחרון משמש validation, עם gap של 20 frames כדי לצמצם זליגת תמונות כמעט זהות בין train ל־validation.
- משתמש ב־DataLoader, batch size של 64 ו־15 epochs.
- פונקציית ההפסד היא `SmoothL1Loss`, עמידה יותר מ־MSE לתגיות steering חריגות.
- האופטימייזר הוא AdamW עם weight decay.
- gradient clipping מונע עדכון קיצוני עקב batch בעייתי.
- `ReduceLROnPlateau` מקטין learning rate כאשר validation מפסיק להשתפר.
- נשמר רק המודל בעל validation loss הטוב ביותר כ־`steering_model_v2.pth`.
- בנוסף ל־loss מחושב validation MAE, שקל יותר להסביר כטעות steering ממוצעת.

### `inference/steering_predictor.py`

טוען את המודל המאומן ומבצע inference בזמן אמת.

- בוחר CUDA אם זמינה, אחרת CPU.
- משתמש בדיוק באותו crop, resize ו־normalization של האימון.
- ממיר BGR שמגיע מ־OpenCV ל־RGB שהמודל מצפה לו.
- מוסיף batch dimension ומפעיל `torch.inference_mode()` כדי לבטל gradients ולחסוך זיכרון וזמן.
- מחזיר float יחיד של steering.
- תומך גם בחיזוי מקובץ תמונה לצורכי בדיקה ידנית.

### `controllers/ai_controller.py`

עוטף את `SteeringPredictor` והופך prediction לפקודת CARLA.

1. prediction מוכפל ב־`STEERING_GAIN`.
2. הערך נחתך למגבלת steering.
3. מופעל exponential smoothing מול הפקודה הקודמת כדי למנוע רעידות חדות.
4. בקר מהירות בסיסי קובע throttle/brake ראשוניים.
5. נוצר `carla.VehicleControl` ונבנה dictionary לצורכי תצוגה.

הבקר אינו קורא בעצמו ל־`apply_control`; הוא מחזיר “פקודה מבוקשת”. בכך הוא מאפשר לשכבות הבטיחות לשנות אותה לפני שהיא מגיעה לרכב.

### `controllers/autopilot_controller.py`

חלופה למודל ה־AI לצורכי השוואה או איסוף נתונים. הוא מפעיל CARLA Autopilot, מבטל החלפות נתיב אקראיות ומחזיר את הפקודה ש־Traffic Manager כבר יצר. הממשק מול `simulation.py` זהה לזה של `AIController`.

### `training/lol.py`

כלי אבחון קטן שמדפיס את גרסת PyTorch, זמינות CUDA, גרסת CUDA ושם ה־GPU. הוא אינו משתתף באימון או בזמן הריצה של הסימולציה.

### קובצי `__init__.py` בתיקיות `training`, `inference` ו־`controllers`

מסמנים את התיקיות כחבילות Python. הם ריקים משום שאין צורך לייצא API נוסף ברמת החבילה.

## 6. ניווט ושליטה במהירות

### `navigation/route_manager.py`

`RouteManager` מנהל מסלול ארוך ומחלק את הנהיגה לשלושה מצבים:

- `AI`: כביש רגיל, מודל ההיגוי מוביל.
- `APPROACH`: התקרבות לצומת או maneuver.
- `INTERSECTION`: הרכב נמצא בתוך צומת.

`plan_new_route()` בוחר יעד אקראי במרחק מינימלי, מעדיף מסלולים הכוללים צומת ומנסה יעד אחר אם אין מסלול. seed קבוע מאפשר התנהגות שחוזרת על עצמה.

אם `GlobalRoutePlanner` של CARLA זמין, נעשה בו שימוש. אחרת מופעל `WaypointGraphPlanner` פנימי המבצע A* על גרף ה־waypoints. עלות המסלול היא המרחק שכבר נסע, וה־heuristic הוא המרחק הישיר ליעד.

`_advance_route_index()` מתקדם רק בין waypoints עוקבים. הדבר מונע קפיצה בטעות לנקודה בצד היציאה של צומת הקרובה גאומטרית לנקודת הכניסה.

`_find_next_maneuver()` מחפש קדימה `LEFT`, `RIGHT` או `STRAIGHT`; `_find_junction_distance()` מחשב מרחק לצומת; ו־`_lookahead_waypoint()` מספק יעד היגוי מספר מטרים קדימה.

ב־planner הפנימי, סוג הפנייה מחושב מהפרש yaw בין הכניסה והיציאה מהצומת: הפרש קטן הוא ישר, חיובי הוא ימין ושלילי הוא שמאל.

### `navigation/intersection_controller.py`

נכנס לפעולה רק בהתקרבות לצומת ובתוכו. הוא מחשב את הזווית מה־ego אל waypoint היעד:

```text
heading_error = target_yaw - vehicle_yaw
route_steering = clip(heading_error × gain)
```

לאחר מכן הוא מערבב בין steering של ה־AI לבין steering של המסלול. משקל ה־AI קטן בצומת, מפני שבאזורים ללא סימוני נתיב המסלול אמין יותר. smoothing נוסף מונע פנייה חדה בתחילת ההתערבות.

### `navigation/road_speed_controller.py`

קורא `vehicle.get_speed_limit()` ומגדיר יעד של 98% מהמהירות החוקית. אם CARLA מחזירה ערך לא תקין, נשמרת המהירות החוקית האחרונה.

בהתקרבות לפנייה בצומת היעד מוקטן באמצעות factor ומגבלה עליונה. ההפרש בין המהירות בפועל ליעד מומר ל־throttle או brake באמצעות gains ו־deadband קטן, שמונע החלפה מתמדת בין גז לברקס.

### `navigation/turn_signals.py`

פונקציה קטנה וטהורה שבוחרת `LEFT`, `RIGHT`, `HAZARD` או `OFF`. אורות חירום מקבלים עדיפות. אחרת מופעל איתות רק כאשר הניווט במצב APPROACH/INTERSECTION ומודיע על פנייה.

### `navigation/__init__.py`

קובץ package מינימלי. אין בו לוגיקה עסקית.

## 7. שכבות הבטיחות

### `safety/safety_layer.py`

מטפל במכשול קדמי. מרחקי ההתערבות דינמיים:

```text
slow_distance      = max(min_slow,      speed_mps × slow_time_gap)
brake_distance     = max(min_brake,     speed_mps × brake_time_gap)
emergency_distance = max(min_emergency, speed_mps × emergency_time_gap)
```

לכן ככל שהרכב מהיר יותר, ההתערבות מתחילה מוקדם יותר.

המצבים הם `CLEAR`, `SLOWING`, `BRAKING`, `EMERGENCY` ו־`CREEPING`. מצב creeping פותר מצב שבו הרכב כמעט נעצר חמישה מטרים מאובייקט ונשאר נעול על brake; הוא מאפשר התקרבות איטית עד מרחק עצירה טבעי.

`is_obstacle_relevant()` משווה road/lane IDs ודוחה רכב בכיוון הנגדי על אותו כביש. בכך משאית בנתיב הנגדי אינה גורמת לבלימה שגויה.

### `safety/lane_keeping.py`

מחשב שני סוגי שגיאה ביחס ל־waypoint הקרוב:

- lateral offset: המכפלה הסקלרית של ההפרש במיקום עם וקטור הימין של הנתיב.
- heading error: הפרש yaw בין הרכב לכיוון הנתיב.

תיקון ההיגוי הוא:

```text
correction = -(offset × lateral_gain + heading_error × heading_gain)
```

התיקון מוגבל ומתווסף ל־steering של ה־AI. המערכת אינה מתערבת במהירות נמוכה או בצומת, כדי לא להילחם בבקר הפנייה.

### `safety/traffic_light_safety.py`

משתמש ב־`vehicle.get_traffic_light()` וב־stop waypoints של הרמזור.

- צהוב יוצר warning.
- באדום מחושב braking range לפי המהירות.
- רחוק מהקו אין התערבות.
- במהירות נמוכה ולפני הקו מותר creeping איטי.
- בהגעה לקו מופעל `red_hold_latched`; גם אם CARLA מפסיקה לדווח את הרמזור מיד אחרי trigger line, הבלם אינו משתחרר והרכב אינו זוחל לתוך הצומת.
- כאשר האור משתנה, ה־state מתאפס.

### `safety/cross_traffic_safety.py`

מזהה האם מסלול רכב אחר עתיד לחצות את מסלול ה־ego.

האלגוריתם מייצג את שני המסלולים כקרניים דו־ממדיות, משתמש ב־cross product למציאת נקודת החיתוך ומחשב:

```text
ego_time   = ego_distance_to_conflict / ego_speed
actor_time = actor_distance_to_conflict / actor_speed
arrival_gap = |ego_time - actor_time|
```

רק actor בטווח, בזווית חצייה מספקת, שנע לכיוון נקודת החיתוך ובתוך time horizon נחשב לסכנה. רכבים שממתינים עם brake או באור אדום מסוננים. פער הגעה בינוני יוצר warning, ופער קטן יחד עם מהירות actor משמעותית גורם לבלימה.

### `safety/blind_spot_safety.py`

מגדיר אזורי שטח מת בקואורדינטות המקומיות של הרכב: עד 10 מטר מאחור, 4 מטר לפנים ובטווח רוחבי המתאים לנתיבים סמוכים.

מיקום כל רכב מומר לציר קדימה וציר ימין לפי yaw של ה־ego. כך הבדיקה עובדת גם כאשר הכביש אינו מקביל לצירי העולם. מוחזרים מצב שמאל/ימין/שניהם, מרחק ו־event key.

ה־Radar מספק מדידה פיזית, ומידע actors של CARLA משמש לסיווג שהאובייקט הוא באמת רכב ולא קיר או מדרכה. בעת Safe Stop, שטח מת ימני תפוס חוסם מעבר לנתיב הימני.

### `safety/inactivity_detector.py`

מכונת מצבים המבוססת על הזמן מאז תשובה תקינה מהבקר:

```text
NORMAL → INACTIVITY_WARNING → SAFE_STOP
```

בעמידה או במהירות נמוכה אין התרעה. לאחר כניסה ל־SAFE_STOP המצב נשאר latched גם כשהרכב נעצר; רק תשובה חדשה מהבקר משחררת אותו. כך העצירה אינה מתבטלת רק משום שהצליחה להוריד את המהירות לאפס.

### `safety/emergency_pull_over.py`

מממש את תגובת החירום המלאה לאובדן בקר/נהג:

```text
MOVING_TO_RIGHT_LANE
→ MOVING_TO_SHOULDER
→ STOPPING_ON_RIGHT
→ STOPPED_WITH_HAZARDS
```

אם הנתיב חסום נוסף מצב `WAITING_FOR_RIGHT_LANE`.

הבקר מחפש lane חוקי מימין מסוג Driving, Shoulder או Parking, ודוחה נתיב נסיעה בכיוון ההפוך. יעד ההיגוי הוא waypoint קדימה בנתיב הרצוי, והפרש heading הופך ל־steering מוגבל.

לפני מעבר נבדק מסדרון גאומטרי מלא: actors דינמיים, הולכי רגל, props, environment objects ו־level bounding boxes. הבדיקה כוללת את הנתיב המיועד, החלק האלכסוני של המיזוג, רכב מאחור ורכב חונה בהמשך. רדיוס ה־bounding box מתווסף לרוחב המסדרון.

אם המסלול אינו פנוי או ששטח המת הימני תפוס, הרכב בולם וממתין. עם הכניסה לשול הוא מתחיל לעצור מיד במקום להמשיך לנסוע לאורכו.

במקביל מוחזרים flags להפעלת hazard lights, צליל wake-up מחזורי, וסימולציית שיחה ל־911 לאחר 120 שניות.

### `safety/alert_manager.py`

מנהל את הצלילים בלי לחסום את לולאת הנהיגה.

- זהות alert מורכבת מהסיבה ומ־event key, ולכן אותו אירוע אינו משמיע צליל בכל frame.
- קיימים warning ו־urgent נפרדים.
- Windows משתמש ב־`winsound` אסינכרוני, macOS ב־`afplay`, ו־Linux מנסה `paplay` או `aplay`.
- אם קובץ חסר, קיים fallback לצליל מערכת או terminal bell.
- ניתן לעצור צליל פעיל כאשר מצב החירום מסתיים.

### `safety/safety_logger.py`

כותב אירועי בטיחות ל־CSV. הוא אינו כותב כל frame, אלא רק כאשר state משתנה, event key חדש מתקבל או מתרחשת התנגשות חדשה. הדבר שומר log קצר ומשמעותי.

כל שורה מכילה זמן מתחילת הנסיעה, מהירות, מרחק מכשול, state, throttle, brake, collision וסוג ה־actor. ב־Free Drive נוצר קובץ ייחודי עם timestamp; בתרחישים ניתן להשתמש בקובץ הכללי.

### `safety/__init__.py`

מסמן את התיקייה כחבילת Python. אין בו לוגיקה.

## 8. תרחישי ההדגמה

### `scenario_catalog.py`

מגדיר את מודל הנתונים של הממשק:

- `TrafficPreset`: חמש רמות תנועה — 0, 10, 20, 30 ו־40 רכבים.
- `ScenarioDefinition`: מזהה, כותרת, תיאור, זמינות וזמן demo.
- `SimulationSettings`: הגדרות ריצה עם validation.
- `SCENARIOS`: שמונת התרחישים שמוצגים למשתמש.

תרחישים מבוקרים מקבלים זמן וכמות תנועה קבועים; רק Free Drive מציג אפשרויות למשתמש.

### `scenarios/scenario_setup.py`

מכיל adapter אחיד בשם `ScenarioRuntime`. לא כל תרחיש צריך את כל היכולות, ולכן ה־adapter בודק בזמן ריצה האם קיימים hooks כגון `update`, `apply_requested_control`, `controller_inactive`, `suppress_lane_keeping`, `force_cross_traffic_detection` ו־`close`.

`setup_scenario()` הוא factory שבונה את המחלקה המתאימה לפי `scenario_id`. כל actors שהתרחיש יוצר מוחזרים ל־`main.py` לצורך cleanup.

### `scenarios/obstacle_ahead.py`

תרחיש בסיס וגם מחלקת עזר לתרחישים אחרים.

- מחפש spawn point על כביש ישר וללא צומת.
- בודק רצף waypoints קדימה כדי לוודא שהכביש נשאר ישר.
- מציב רכב 30–40 מטר לפנים.
- מבטל physics ומפעיל hand brake כדי ליצור מכשול יציב.
- כולל בחירת blueprint, הגבהת transform ועצירת ego.

### `scenarios/lead_vehicle_emergency_brake.py`

יורש מכלי העזר של `ObstacleAheadScenario`. רכב מוביל מתחיל 16 מטר לפנים עם Autopilot. לאחר ארבע שניות ה־Autopilot מבוטל ונשלחת בלימת 100%, כדי לבדוק תגובה לשינוי חד ולא רק לאובייקט סטטי.

### `scenarios/vehicle_cut_in.py`

ממקם רכב בנתיב השמאלי על קטע דו־נתיבי ישר.

- בשלב ההמתנה הוא נוסע לאט, כדי שה־ego ישיג אותו בזמן סביר.
- החיתוך מתחיל רק כאשר ה־ego נע והרכב השני נמצא בפער longitudinal מוגדר לפנים.
- ברגע ההפעלה נבנה מסלול חדש מהמיקום הנוכחי: קטע קצר בשמאל, יעד אלכסוני בנתיב ה־ego ולאחריו waypoints ישרים.
- בקר heading מוגבל מונע מהרכב להסתובב או לאבד שליטה.
- בזמן החיתוך מהירות הרכב עולה, כך שהאירוע אגרסיבי אך ניתן למניעה.
- רמזורים סמוכים מוקפאים זמנית על ירוק כדי שלא יעכבו את הדמו, ומוחזרים למצבם המקורי ב־`close()`.

### `scenarios/cross_traffic.py`

מאתר קבוצת רמזורים עם שתי גישות בזווית חוצה. ה־ego ממוקם לפני גישה אחת ורכב נוסף לפני הגישה השנייה.

הרכב החוצה מוחזק עד שה־ego מתקרב. מהירות החצייה מחושבת לפי זמן ההגעה המשוער של ה־ego לנקודת הקונפליקט, כדי ליצור סכנה אמיתית. הרכב החוצה מונחה על מסלול waypoints עם בקר heading ומהירות, במקום velocity קבוע שעלול לגרום לו להסתובב. הרמזור של ה־ego מוקפא על ירוק ומוחזר בסיום.

### `scenarios/lane_departure.py`

לאחר ארבע שניות מזריק steering ימינה. עוצמת ההזרקה תלויה במרחק שנותר ל־lateral offset הרצוי ומוגבלת לטווח מתון.

במהלך יצירת הסטייה, Lane Keeping מושבת בכוונה כדי לאפשר לאירוע להתרחש. כאשר Lane Invasion Sensor מדווח על חציית קו, ההזרקה נעצרת ושכבת שמירת הנתיב משתחררת לתקן את הרכב.

### `scenarios/red_traffic_light.py`

מאתר רמזור עם stop waypoint שמאפשר להציב את ה־ego 28 מטר לפניו. מצב הרמזור נשמר, מוקפא על אדום ומוחזר בסיום. כך מתקבל demo שחוזר על עצמו ואינו תלוי במחזור הרמזור האקראי.

### `scenarios/driver_inactivity.py`

בוחר קטע ישר עם נתיב/שול חוקי מימין. לאחר ארבע שניות hook בשם `controller_inactive()` מחזיר true, ולכן `simulation.py` מפסיק לבקש פקודות חדשות מהבקר. ה־Inactivity Detector עובר מאזהרה ל־Safe Stop ומפעיל את תהליך הירידה לשול.

### `scenarios/__init__.py`

מייצא את `setup_scenario`, כך ש־`main.py` אינו צריך להכיר את מבנה הקבצים הפנימי.

## 9. לולאת הסימולציה והתצוגה

### `simulation.py`

זהו קובץ האינטגרציה המרכזי.

`run_simulation()` יוצר את כל שכבות הבטיחות והניווט ומנהל את ה־state שלהן. הלולאה ממתינה ל־tick של CARLA, אך מפעילה inference רק כאשר מתקבל camera frame חדש.

כאשר הבקר זורק exception, נשמרת הפקודה התקינה האחרונה וה־Inactivity Detector מתחיל לספור. אם מעולם לא התקבלה פקודה תקינה, נשלחת מיד פקודת בלימה מלאה.

הקובץ אחראי גם על:

- עדכון מצלמת spectator מאחורי הרכב.
- חישוב מהירות וניווט.
- הפעלת שכבות הבטיחות בסדר קבוע.
- suppression של Lane Keeping בזמן צומת, בלימת חירום או תרחיש מבוקר.
- אורות איתות ואורות חירום באמצעות `VehicleLightState`.
- איחוד states רבים למחרוזת אירוע אחת.
- תרגום state לסיבה אנושית שמוצגת ב־banner ומשמיעה צליל.
- יצירת event keys כדי למנוע alerts ו־log כפולים.
- ציור overlay ב־OpenCV עם steering, מהירות, מסלול, מכשול, רמזור, תנועה חוצה, שטח מת וסטטוס חירום.
- hotkeys: `I` לחוסר פעילות, `L` ל־Lane Keeping, `N` למסלול חדש ו־Q/Esc לעצירה.
- קבלת אותן פקודות מהממשק דרך קובץ IPC.
- עדכון `JourneyEvaluator` רק ב־Free Drive.
- הדפסת `JOURNEY_RESULT` כ־JSON ונתיב ה־log כדי שהממשק יוכל לקרוא אותם.
- shutdown עצמאי של האיתות, הצלילים, logger והבקר.

### `journey_evaluator.py`

אוסף מדדים בכל frame ומחשב בסיום ציון מתוך 100:

| רכיב | מקסימום | חישוב |
|---|---:|---|
| Safety | 50 | פחות 40 לכל collision ופחות 3 לכל emergency intervention |
| Lane discipline | 20 | פחות 5 לכל אירוע lane departure ייחודי |
| Speed efficiency | 20 | היחס הממוצע למהירות היעד, ללא זמנים שבהם safety חייב עצירה |
| Completion | 10 | 10 לסיום מלא, 5 לעצירת משתמש, 0 לשגיאה |

המרחק מחושב בין מיקומים עוקבים; קפיצה מעל 10 מטר נחשבת teleport ואינה נספרת. collisions נספרים בקצה עולה בלבד, ו־lane events נשמרים ב־set כדי למנוע ספירה כפולה.

מלבד score ו־rating מוחזרים פירוט הנקודות והסבר מילולי: מדוע ירדו נקודות, מה הייתה יעילות המהירות וכיצד הסתיימה הנסיעה. חשוב להציג את הציון כמדד פנימי להשוואת ריצות, לא כתקן בטיחות רשמי.

### `interface.py`

ממשק Tkinter באנגלית, המעוצב כ־dashboard כהה.

- מסך פתיחה ממותג עם מספר scenarios ושכבות בטיחות.
- About, בחירת scenario, הגדרות Free Drive ומסך confirmation.
- כרטיסי scenario ממוספרים עם hover וסטטוס זמינות.
- Free Drive בלבד מאפשר בחירת זמן וצפיפות תנועה.
- הסימולציה מופעלת באמצעות `subprocess.Popen`, ולכן ה־GUI נשאר responsive.
- thread רקע קורא stdout ומכניס שורות ל־`queue.Queue`; רק ה־thread הראשי משנה widgets של Tkinter.
- prefixes מוסכמים כגון `CURRENT_EVENT`, `SCENARIO_EVENT`, `TURN_SIGNAL`, `JOURNEY_RESULT` ו־`SIMULATION_RESULT` הופכים output טקסטואלי ל־UI מובנה.
- במסך הריצה מוצגים connection status, האירוע הפעיל, איתות, console וכפתורי שליטה.
- בסיום Free Drive מוצג score card עם ארבעת מרכיבי הציון, progress bars והסבר מילולי מלא.
- כפתור Save Log מעתיק את קובץ ה־CSV למיקום שהמשתמש בוחר.
- עצירה רגילה נעשית באמצעות signal file; אם התהליך אינו נסגר בתוך חמש שניות, מופעל terminate כ־fallback.

## 10. קובצי הצלילים

### `assets/sounds/generate_simulated_call.py`

יוצר מקומית WAV סטריאופוני בקצב 44.1kHz. הפונקציה `tone()` מחברת שני תדרי DTMF לכל ספרה, מוסיפה fade קצר למניעת click, ו־`silence()` מוסיפה רווחים. הקובץ מנגן 9־1־1 ולאחר מכן fragment של ringback. זו סימולציה בלבד ואינה מחייגת לשירות חיצוני.

### `assets/sounds/README.md`

מתעד את מקור ורישיון קובצי הקול. צלילי ADAS, אזעקה וצפירה הם CC0; צליל השיחה נוצר בתוך הפרויקט.

### קובצי WAV

- `adas_warning.wav`: התראה רגילה.
- `adas_urgent.wav`: התערבות דחופה.
- `driver_wakeup_alarm.wav`: צליל מחזורי להעיר נהג לא מגיב.
- `emergency_horn.wav`: צליל צפירת רכב חלופי.
- `simulated_call.wav`: חיוג 911 מדומה וצליל התחברות.

## 11. דרכי הפעלה והטמעה

### א. הפעלה מקומית — הדרך הנוכחית והמומלצת להצגה

1. מתקינים CARLA בגרסה המתאימה ל־Python API שבסביבה.
2. מתקינים Python dependencies: `carla`, `numpy`, `opencv-python`, `torch`, `torchvision`, `Pillow` ו־`pandas`.
3. מוודאים שקובץ המודל נמצא תחת `trained_models/steering_model_v2.pth`.
4. מפעילים את שרת CARLA:

```powershell
.\CarlaUE4.exe -carla-rpc-port=2000
```

5. ממתינים לטעינת העולם ומפעילים:

```powershell
python interface.py
```

אפשר גם להפעיל ללא GUI:

```powershell
python main.py --scenario free_drive --duration-minutes 5 --traffic-vehicles 20
```

### ב. CARLA על מחשב נפרד

ניתן להריץ את ה־simulator במחשב חזק עם GPU ואת קוד Python במחשב אחר. משנים ב־`config.py` את `HOST` לכתובת ה־IP של שרת CARLA, פותחים את פורט 2000 ואת פורט ה־streaming הנלווה ברשת, ומשאירים את שאר הארכיטקטורה ללא שינוי.

### ג. אריזה כאפליקציית Windows

ניתן להשתמש ב־PyInstaller עבור `interface.py`, אך יש לכלול במפורש:

- קובצי WAV תחת `assets/sounds`.
- קובץ המודל תחת `trained_models`.
- DLLs ותלויות של PyTorch/OpenCV לפי סביבת Windows.
- CARLA Python package תואם.

CARLA עצמה נשארת תהליך שרת חיצוני; אין צורך לארוז את Unreal Engine בתוך executable של הממשק.

### ד. הטמעה עתידית ברכב אמיתי

הארכיטקטורה מאפשרת החלפת adapters:

- `sensors.py` יוחלף בדרייברים למצלמה, Radar, CAN ו־GPS אמיתיים.
- `vehicles.py` ו־`carla.VehicleControl` יוחלפו ב־vehicle interface השולח פקודות למערכת drive-by-wire.
- `RouteManager` יחובר למפות ולוקליזציה אמיתיות.
- שכבות הבטיחות יכולות להישאר כרעיון, אך ידרשו כיול, redundancy, בדיקות חומרה ועמידה בתקנים.

הקוד הנוכחי הוא אב־טיפוס סימולטיבי ואינו מיועד להפעלה ברכב אמיתי ללא תהליך safety engineering ואימות פורמלי.

## 12. תיקיות וקבצים שנוצרים בזמן עבודה

- `dataset/images/`: תמונות האימון.
- `dataset/driving_log.csv`: תגיות הנהיגה.
- `trained_models/steering_model_v2.pth`: משקולות המודל.
- `safety_logs/`: אירועי בטיחות וקובצי Free Drive בעלי timestamp.
- `__pycache__/`: bytecode מקומי של Python; אינו חלק מקוד המקור ונמצא ב־`.gitignore`.

## 13. נקודות מרכזיות להצגה

1. המודל אינו מקבל שליטה בלעדית על הרכב; הוא מספק steering מבוקש בלבד.
2. כל פקודה עוברת דרך שכבות בטיחות עצמאיות לפני `apply_control`.
3. בצמתים נעשה מעבר הדרגתי מראייה למסלול גאומטרי.
4. מרחקי הבלימה תלויים במהירות ולא נשענים על threshold קבוע בלבד.
5. תנועה חוצה מזוהה באמצעות חיזוי זמן הגעה לנקודת קונפליקט.
6. Safe Stop הוא תהליך רב־שלבי הכולל בדיקת נתיב ושול, שטח מת, hazards, אזעקה ושיחה מדומה.
7. כל התערבות ניתנת להסבר למשתמש, נשמרת ב־log ומשפיעה על הערכת הנסיעה.
8. שמונת התרחישים מספקים הדגמה חוזרת של מצבי הסיכון העיקריים.

## 14. מגבלות שחשוב לומר בכנות

- זיהוי רמזורים, מיקום actors ו־lane geometry משתמשים ב־ground truth של CARLA ולא במודל perception ממצלמה.
- מודל ה־CNN אומן ל־steering בלבד; הוא אינו מחליט על מסלול, מהירות או בלימה.
- איכות הנהיגה תלויה בכיסוי ובאיזון של dataset האימון.
- הציון הוא כלי הערכה פנימי לפרויקט, לא מדד בטיחות מוסמך.
- התנהגות בעולם אמיתי תדרוש חיישנים אמיתיים, sensor fusion, localization, fail-operational hardware ותהליך validation רחב בהרבה.

מגבלות אלו אינן סותרות את מטרת הפרויקט: להדגים בצורה ברורה כיצד AI לנהיגה יכול לעבוד יחד עם שכבות הגנה הניתנות להסבר ולמנוע ממנו לקבל לבדו החלטות בטיחות קריטיות.
