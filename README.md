# CodeBall
Russian AI Cup 2018 (6th in final)

В самом начале конкурса оптимизировал симулятор, чтобы быстро посчитать, где и с какой скоростью окажусь, если буду ехать в заданном направлении t тиков (функция Simulator::GetRobotPVContainer -  на мой взгляд, здесь самое интересное, что я сделал за весь конкурс). Т.е. изменение координаты за 1 тик, которое симулятор считает за 100 микротиков, можно посчитать одной формулой..

Расчет положения мяча и летящих роботов на протяжении 100 тиков реализован в функции MyStrategy::InitBallEntities. Чужие роботы предполагались летящими без нитры. 

Перебирал я время стояния + направления движения + время начала прыжка. 

Дальше, использовав уравнения движения мяча и прыгнувшего робота, рассчитывал время коллизии (без нитро вычисляется вообще легко - Simulator::GetCollisionT). 
Определял положения мяча и робота за это время, потом симулятором считал коллизию - MyStrategy::SimulateFullCollision. Здесь кроме своего робота, который планирует прыгнуть, учитывал также всех роботов, находящихся в воздухе - это неплохо помогало, особенно в защите, когда я бил не по мячу, а по чужому роботу, который уже выносил мяч. Повторюсь, чужой робот предполагался летящим без нитры, иначе его траекторию невозможно предсказать. 

Ну и сравнивал получившиеся варианты удара по мячу -  MyStrategy::CompareBeContainers, MyStrategy::CompareDefenderBallEntities. К этим функциям написал подробные комментарии в коде.

Прыжок с нитрой можно быстро и абсолютно точно просимулировать, если задать target_velocity_x и target_velocity_z равными текущим соответствующим составляющим скорости робота. А target_velocity_y устремить максимально вверх. Тогда робот будет лететь с постоянными составляющими скорости по x,y,z. Такой подход, конечно, сужает возможности использования нитро, зато отлично симулируется. Прыжок робота на мяч с нитро описан в функции MyStrategy::simulate_ball_nitro_jump. Здесь отмечу параметр stopNitroTick, который отвечал за время отключения нитро. Его перебор добавил вариативности.