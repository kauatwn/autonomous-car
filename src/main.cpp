/*
 * Carro Robótico Autônomo com Arduino Uno
 *
 * Descrição do Projeto:
 * Firmware para navegação autônoma e desvio de obstáculos utilizando sensor de distância ultrassônico.
 * O sistema avalia continuamente a distância frontal em tempo real por tempo de voo acústico,
 * controla a tração diferencial de dois motores CC acionados por circuito integrado Ponte H (L293D)
 * e gerencia sinalizações visuais (LEDs) e sonora (buzzer) conforme a faixa de proximidade detectada,
 * transmitindo dados de telemetria periodicamente via porta serial.
 *
 * Regras de Negócio e Comportamento Operacional:
 * 1. Detecção de Distância (Sensor Ultrassônico HC-SR04):
 *    - Disparo de pulso de excitação de 10 microssegundos no pino D2 (Trigger).
 *    - Mensuração do tempo de voo do eco acústico no pino D3 (Echo) com timeout de 25000 microssegundos.
 *    - Conversão da duração do eco em distância linear (cm) considerando a velocidade do som no ar (343 m/s).
 * 2. Controle de Navegação e Tração Diferencial (Ponte H L293D nos pinos D4 a D7):
 *    - Distância >= 30.0 cm (Via Livre): Deslocamento retilíneo contínuo para frente.
 *    - 15.0 cm <= Distância < 30.0 cm (Desvio de Obstáculo): Manobra evasiva com giro diferencial sobre o eixo.
 *    - Distância < 15.0 cm (Proximidade Crítica): Frenagem de emergência e marcha a ré de segurança.
 * 3. Sinalização Visual (LEDs) e Sonora (Buzzer):
 *    - LED Verde (D8): Via livre desobstruída (carro em avanço).
 *    - LED Amarelo (D9): Execução de manobra de desvio.
 *    - LED Vermelho (D10): Obstáculo crítico / marcha a ré de segurança.
 *    - Buzzer (D11): Alarme sonoro pulsado/contínuo ativo durante proximidade crítica.
 */

#include <Arduino.h>

// Mapeamento de pinos do hardware
constexpr uint8_t pin_ultrasonic_trigger = 2;    // Disparo do sensor ultrassônico (Trigger)
constexpr uint8_t pin_ultrasonic_echo = 3;       // Entrada digital: eco do sensor ultrassônico (Echo)
constexpr uint8_t pin_motor_left_forward = 4;    // Motor esquerdo para frente (L293D IN1)
constexpr uint8_t pin_motor_left_backward = 5;   // Motor esquerdo para trás (L293D IN2)
constexpr uint8_t pin_motor_right_backward = 6;  // Motor direito para trás (L293D IN3 - compensação do chassi)
constexpr uint8_t pin_motor_right_forward = 7;   // motor direito para frente (L293D IN4 - compensação do chassi)
constexpr uint8_t pin_led_free = 8;              // LED verde (via livre / avanço)
constexpr uint8_t pin_led_turning = 9;           // LED amarelo (manobra de desvio)
constexpr uint8_t pin_led_critical = 10;         // LED vermelho (obstáculo crítico / ré)
constexpr uint8_t pin_buzzer = 11;               // Buzzer piezoelétrico de alerta

// Parâmetros físicos de propagação acústica e limites do sensor HC-SR04
constexpr unsigned long ultrasonic_timeout_us = 25000;  // Timeout de 25 ms (~4.2 metros máximo)
constexpr float speed_of_sound_cm_per_us = 0.0343F;     // Velocidade do som no ar: 343 m/s = 0.0343 cm/us
constexpr float min_valid_distance_cm = 2.0F;           // Limite físico inferior de mensuração do sensor
constexpr float max_valid_distance_cm = 400.0F;         // Limite físico superior de mensuração do sensor

// Limiares operacionais de navegação e margem de segurança (cm)
constexpr float distance_safe_threshold_cm = 30.0F;      // Acima ou igual: pista livre para progressão
constexpr float distance_critical_threshold_cm = 15.0F;  // Abaixo: risco iminente de colisão, exige ré

// Parâmetros de temporização, frequência acústica e comunicação serial
constexpr unsigned long serial_baud_rate = 9600;          // Taxa de transmissão da porta serial (9600 bps)
constexpr unsigned long telemetry_interval_ms = 1000;     // Intervalo de telemetria serial (1 segundo)
constexpr unsigned long sampling_interval_ms = 100;       // Intervalo de leitura periódica do sensor (100 ms)
constexpr unsigned int buzzer_alarm_frequency_hz = 1200;  // Frequência acústica do alarme (1200 Hz)
constexpr uint8_t telemetry_decimals = 1;                 // Precisão decimal da telemetria de distância

// Estados operacionais da máquina de navegação autônoma
constexpr uint8_t state_free = 0;      // Pista desobstruída: deslocamento retilíneo para frente
constexpr uint8_t state_turn = 1;      // Obstáculo detectado: manobra evasiva de giro sobre o eixo
constexpr uint8_t state_critical = 2;  // Proximidade crítica: parada e marcha a ré de segurança

// Variáveis de estado global do sistema
static uint8_t current_state = state_free;
static float current_distance_cm = max_valid_distance_cm;
static unsigned long last_telemetry_ms = 0;
static unsigned long last_sampling_ms = 0;

// Emite o pulso no pino Trigger, afere o tempo de voo no pino Echo e calcula a distância linear
static float read_ultrasonic_distance_cm() {
  digitalWrite(pin_ultrasonic_trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(pin_ultrasonic_trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin_ultrasonic_trigger, LOW);

  const unsigned long echo_duration_us = pulseIn(pin_ultrasonic_echo, HIGH, ultrasonic_timeout_us);

  // Tratamento de perda de eco / fora de alcance físico
  if (echo_duration_us == 0) {
    return max_valid_distance_cm;
  }

  // Cálculo com base no tempo de ida e volta do pulso de som: Distância = (Tempo * v) / 2
  const float calculated_distance = static_cast<float>(echo_duration_us) * speed_of_sound_cm_per_us / 2.0F;

  if (calculated_distance > max_valid_distance_cm) {
    return max_valid_distance_cm;
  }
  if (calculated_distance < min_valid_distance_cm) {
    return min_valid_distance_cm;
  }
  return calculated_distance;
}

// Avalia se o trajeto à frente está desobstruído para progressão retilínea
static bool is_path_clear(const float distance_cm) { return distance_cm >= distance_safe_threshold_cm; }

// Avalia se a proximidade do obstáculo demanda parada emergencial e manobra de ré
static bool is_obstacle_critical(const float distance_cm) { return distance_cm < distance_critical_threshold_cm; }

// Determina o estado operacional da máquina de estados com base na distância linear medida
static uint8_t determine_navigation_state(const float distance_cm) {
  if (is_obstacle_critical(distance_cm)) {
    return state_critical;
  }
  if (is_path_clear(distance_cm)) {
    return state_free;
  }
  return state_turn;
}

// Desativa todos os canais de excitação da ponte H L293D
static void stop_drive_motors() {
  digitalWrite(pin_motor_left_forward, LOW);
  digitalWrite(pin_motor_left_backward, LOW);
  digitalWrite(pin_motor_right_forward, LOW);
  digitalWrite(pin_motor_right_backward, LOW);
}

// Aplica os níveis lógicos da ponte H para acionamento diferencial dos motores
static void control_drive_motors(const uint8_t state) {
  switch (state) {
    case state_free:
      // Avanço em linha reta: ambos os motores tracionam para frente
      digitalWrite(pin_motor_left_forward, HIGH);
      digitalWrite(pin_motor_left_backward, LOW);
      digitalWrite(pin_motor_right_forward, HIGH);
      digitalWrite(pin_motor_right_backward, LOW);
      break;

    case state_turn:
      // Manobra evasiva: giro diferencial sobre o eixo (esquerda reverte, direita avança)
      digitalWrite(pin_motor_left_forward, LOW);
      digitalWrite(pin_motor_left_backward, HIGH);
      digitalWrite(pin_motor_right_forward, HIGH);
      digitalWrite(pin_motor_right_backward, LOW);
      break;

    case state_critical:
      // Marcha a ré de segurança: ambos os motores tracionam em sentido reverso
      digitalWrite(pin_motor_left_forward, LOW);
      digitalWrite(pin_motor_left_backward, HIGH);
      digitalWrite(pin_motor_right_forward, LOW);
      digitalWrite(pin_motor_right_backward, HIGH);
      break;

    default:
      stop_drive_motors();
      break;
  }
}

// Atualização das saídas digitais dos LEDs de sinalização visual
static void update_visual_signaling(const uint8_t state) {
  switch (state) {
    case state_free:
      digitalWrite(pin_led_free, HIGH);
      digitalWrite(pin_led_turning, LOW);
      digitalWrite(pin_led_critical, LOW);
      break;

    case state_turn:
      digitalWrite(pin_led_free, LOW);
      digitalWrite(pin_led_turning, HIGH);
      digitalWrite(pin_led_critical, LOW);
      break;

    case state_critical:
      digitalWrite(pin_led_free, LOW);
      digitalWrite(pin_led_turning, LOW);
      digitalWrite(pin_led_critical, HIGH);
      break;

    default:
      digitalWrite(pin_led_free, LOW);
      digitalWrite(pin_led_turning, LOW);
      digitalWrite(pin_led_critical, LOW);
      break;
  }
}

// Acionamento do buzzer piezoelétrico para emissão de alerta sonoro
static void control_acoustic_alarm(const bool activate) {
  if (activate) {
    tone(pin_buzzer, buzzer_alarm_frequency_hz);
    return;
  }
  noTone(pin_buzzer);
}

// Retorna a descrição textual do estado de navegação para a telemetria serial
static const __FlashStringHelper* get_navigation_state_label(const uint8_t state) {
  switch (state) {
    case state_free:
      return F("VIA LIVRE (AVANÇANDO)");
    case state_turn:
      return F("OBSTÁCULO DETECTADO (DESVIO / GIRO)");
    case state_critical:
      return F("PROXIMIDADE CRÍTICA (MARCHA A RÉ)");
    default:
      break;
  }
  return F("INDEFINIDO");
}

// Retorna a descrição textual do acionamento dos motores para a telemetria serial
static const __FlashStringHelper* get_motor_action_label(const uint8_t state) {
  switch (state) {
    case state_free:
      return F("FRENTE / FRENTE");
    case state_turn:
      return F("MARCHA A RÉ / FRENTE");
    case state_critical:
      return F("MARCHA A RÉ / MARCHA A RÉ");
    default:
      break;
  }
  return F("PARADO");
}

// Transmissão periódica das informações do veículo pela porta serial
static void transmit_telemetry() {
  Serial.print(F("[TELEMETRIA] Distância: "));
  Serial.print(current_distance_cm, telemetry_decimals);
  Serial.print(F(" cm | Estado: "));
  Serial.print(get_navigation_state_label(current_state));
  Serial.print(F(" | Motores: "));
  Serial.println(get_motor_action_label(current_state));
}

void setup() {
  Serial.begin(serial_baud_rate);
  Serial.println(F("=================================================="));
  Serial.println(F(" CARRO ROBÔTICO AUTÔNOMO - ARDUINO UNO            "));
  Serial.println(F(" Status: Inicializado com Sucesso                 "));
  Serial.println(F("=================================================="));

  pinMode(pin_ultrasonic_trigger, OUTPUT);
  pinMode(pin_ultrasonic_echo, INPUT);

  pinMode(pin_motor_left_forward, OUTPUT);
  pinMode(pin_motor_left_backward, OUTPUT);
  pinMode(pin_motor_right_forward, OUTPUT);
  pinMode(pin_motor_right_backward, OUTPUT);

  pinMode(pin_led_free, OUTPUT);
  pinMode(pin_led_turning, OUTPUT);
  pinMode(pin_led_critical, OUTPUT);
  pinMode(pin_buzzer, OUTPUT);

  // Inicialização dos atuadores em condição segura
  stop_drive_motors();
  control_acoustic_alarm(false);

  // Amostragem inicial e partida determinística
  current_distance_cm = read_ultrasonic_distance_cm();
  current_state = determine_navigation_state(current_distance_cm);

  control_drive_motors(current_state);
  update_visual_signaling(current_state);
  control_acoustic_alarm(current_state == state_critical);
}

void loop() {
  const unsigned long current_ms = millis();

  // Amostragem periódica do sensor ultrassônico a cada 100 ms
  if (current_ms - last_sampling_ms >= sampling_interval_ms) {
    last_sampling_ms = current_ms;

    current_distance_cm = read_ultrasonic_distance_cm();
    current_state = determine_navigation_state(current_distance_cm);

    // Atualização imediata dos motores, LEDs e alarme sonoro
    control_drive_motors(current_state);
    update_visual_signaling(current_state);
    control_acoustic_alarm(current_state == state_critical);
  }

  // Transmissão periódica da telemetria serial a cada 1 segundo (1000 ms)
  if (current_ms - last_telemetry_ms >= telemetry_interval_ms) {
    last_telemetry_ms = current_ms;
    transmit_telemetry();
  }
}
