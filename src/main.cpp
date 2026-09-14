/*
 * Carro Robotico Autonomo com Arduino Uno
 *
 * Descricao do Projeto:
 * Firmware para navegacao autonoma e desvio de obstaculos utilizando sensor de distancia ultrassonico.
 * O sistema avalia continuamente a distancia frontal em tempo real por tempo de voo acustico,
 * controla a tracao diferencial de dois motores CC acionados por circuito integrado Ponte H (L293D)
 * e gerencia sinalizacoes visuais (LEDs) e sonora (buzzer) conforme a faixa de proximidade detectada,
 * transmitindo dados de telemetria periodicamente via porta serial.
 *
 * Regras de Negocio e Comportamento Operacional:
 * 1. Deteccao de Distancia (Sensor Ultrassonico HC-SR04):
 *    - Disparo de pulso de excitacao de 10 microssegundos no pino D2 (Trigger).
 *    - Mensuracao do tempo de voo do eco acustico no pino D3 (Echo) com timeout de 25000 microssegundos.
 *    - Conversao da duracao do eco em distancia linear (cm) considerando a velocidade do som no ar (343 m/s).
 * 2. Controle de Navegacao e Tracao Diferencial (Ponte H L293D nos pinos D4 a D7):
 *    - Distancia >= 30.0 cm (Via Livre): Deslocamento retilineo contínuo para frente.
 *    - 15.0 cm <= Distancia < 30.0 cm (Desvio de Obstaculo): Manobra evasiva com giro diferencial sobre o eixo.
 *    - Distancia < 15.0 cm (Proximidade Critica): Frenagem de emergencia e marcha a re de seguranca.
 * 3. Sinalizacao Visual (LEDs) e Sonora (Buzzer):
 *    - LED Verde (D8): Via livre desobstruida (carro em avanco).
 *    - LED Amarelo (D9): Execucao de manobra de desvio.
 *    - LED Vermelho (D10): Obstaculo critico / marcha a re de seguranca.
 *    - Buzzer (D11): Alarme sonoro pulsado/continuo ativo durante proximidade critica.
 */

#include <Arduino.h>

namespace {
// Mapeamento de pinos do hardware
constexpr uint8_t pin_ultrasonic_trig = 2;       // Saida digital: disparo do sensor ultrassonico (Trigger)
constexpr uint8_t pin_ultrasonic_echo = 3;       // Entrada digital: eco do sensor ultrassonico (Echo)
constexpr uint8_t pin_motor_left_forward = 4;    // Saida digital: motor esquerdo para frente (L293D IN1)
constexpr uint8_t pin_motor_left_backward = 5;   // Saida digital: motor esquerdo para tras (L293D IN2)
constexpr uint8_t pin_motor_right_forward = 6;   // Saida digital: motor direito para frente (L293D IN3)
constexpr uint8_t pin_motor_right_backward = 7;  // Saida digital: motor direito para tras (L293D IN4)
constexpr uint8_t pin_led_free = 8;              // Saida digital: LED verde (via livre / avanco)
constexpr uint8_t pin_led_turning = 9;           // Saida digital: LED amarelo (manobra de desvio)
constexpr uint8_t pin_led_critical = 10;         // Saida digital: LED vermelho (obstaculo critico / re)
constexpr uint8_t pin_buzzer = 11;               // Saida digital: buzzer piezoeletrico de alerta

// Parametros fisicos de propagacao acustica e limites do sensor HC-SR04
constexpr unsigned long ultrasonic_timeout_us = 25000;  // Timeout de 25 ms (~4.2 metros maximo)
constexpr float speed_of_sound_cm_per_us = 0.0343F;     // Velocidade do som no ar: 343 m/s = 0.0343 cm/us
constexpr float min_valid_distance_cm = 2.0F;           // Limite fisico inferior de mensuracao do sensor
constexpr float max_valid_distance_cm = 400.0F;         // Limite fisico superior de mensuracao do sensor

// Limiares operacionais de navegacao e margem de seguranca (cm)
constexpr float distance_safe_threshold_cm = 30.0F;      // Acima ou igual: pista livre para progressao
constexpr float distance_critical_threshold_cm = 15.0F;  // Abaixo: risco iminente de colisão, exige re

// Parametros de temporizacao, frequencia acustica e comunicacao serial
constexpr unsigned long serial_baud_rate = 9600;          // Taxa de transmissao da porta serial (9600 bps)
constexpr unsigned long telemetry_interval_ms = 1000;     // Intervalo de telemetria serial (1 segundo)
constexpr unsigned long sampling_interval_ms = 100;       // Intervalo de leitura periodica do sensor (100 ms)
constexpr unsigned int buzzer_alarm_frequency_hz = 1200;  // Frequencia acustica do alarme (1200 Hz)
constexpr uint8_t telemetry_decimals = 1;                 // Precisao decimal da telemetria de distancia

// Estados operacionais da maquina de navegacao autonoma
enum class NavigationState : uint8_t {
  Free,      // Pista desobstruida: deslocamento retilineo para frente
  Turn,      // Obstaculo detectado: manobra evasiva de giro sobre o eixo
  Critical,  // Proximidade critica: parada e marcha a re de seguranca
};

// Variaveis de estado global do sistema
auto current_state = NavigationState::Free;
float current_distance_cm = max_valid_distance_cm;
unsigned long last_telemetry_ms = 0;
unsigned long last_sampling_ms = 0;

// Emite o pulso no pino Trigger, afere o tempo de voo no pino Echo e calcula a distancia linear
float read_ultrasonic_distance_cm() {
  digitalWrite(pin_ultrasonic_trig, LOW);
  delayMicroseconds(2);
  digitalWrite(pin_ultrasonic_trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin_ultrasonic_trig, LOW);

  const unsigned long echo_duration_us = pulseIn(pin_ultrasonic_echo, HIGH, ultrasonic_timeout_us);

  // Tratamento de perda de eco / fora de alcance fisico
  if (echo_duration_us == 0) {
    return max_valid_distance_cm;
  }

  // Calculo com base no tempo de ida e volta do pulso de som: Distancia = (Tempo * v) / 2
  const float calculated_distance = static_cast<float>(echo_duration_us) * speed_of_sound_cm_per_us / 2.0F;

  if (calculated_distance > max_valid_distance_cm) {
    return max_valid_distance_cm;
  }
  if (calculated_distance < min_valid_distance_cm) {
    return min_valid_distance_cm;
  }
  return calculated_distance;
}

// Avalia se o trajeto a frente esta desobstruido para progressao retilinea
bool is_path_clear(const float distance_cm) { return distance_cm >= distance_safe_threshold_cm; }

// Avalia se a proximidade do obstaculo demanda parada emergencial e manobra de re
bool is_obstacle_critical(const float distance_cm) { return distance_cm < distance_critical_threshold_cm; }

// Determina o estado operacional da maquina de estados com base na distancia linear medida
NavigationState determine_navigation_state(const float distance_cm) {
  if (is_obstacle_critical(distance_cm)) {
    return NavigationState::Critical;
  }
  if (is_path_clear(distance_cm)) {
    return NavigationState::Free;
  }
  return NavigationState::Turn;
}

// Desativa todos os canais de excitacao da ponte H L293D
void stop_drive_motors() {
  digitalWrite(pin_motor_left_forward, LOW);
  digitalWrite(pin_motor_left_backward, LOW);
  digitalWrite(pin_motor_right_forward, LOW);
  digitalWrite(pin_motor_right_backward, LOW);
}

// Aplica os niveis logicos da ponte H para acionamento diferencial dos motores
void control_drive_motors(const NavigationState state) {
  switch (state) {
    case NavigationState::Free:
      // Avanco em linha reta: ambos os motores tracionam para frente
      digitalWrite(pin_motor_left_forward, HIGH);
      digitalWrite(pin_motor_left_backward, LOW);
      digitalWrite(pin_motor_right_forward, HIGH);
      digitalWrite(pin_motor_right_backward, LOW);
      break;

    case NavigationState::Turn:
      // Manobra evasiva: giro diferencial sobre o eixo (esquerda reverte, direita avanca)
      digitalWrite(pin_motor_left_forward, LOW);
      digitalWrite(pin_motor_left_backward, HIGH);
      digitalWrite(pin_motor_right_forward, HIGH);
      digitalWrite(pin_motor_right_backward, LOW);
      break;

    case NavigationState::Critical:
      // Marcha a re de seguranca: ambos os motores tracionam em sentido reverso
      digitalWrite(pin_motor_left_forward, LOW);
      digitalWrite(pin_motor_left_backward, HIGH);
      digitalWrite(pin_motor_right_forward, LOW);
      digitalWrite(pin_motor_right_backward, HIGH);
      break;
  }
}

// Atualizacao das saidas digitais dos LEDs de sinalizacao visual
void update_visual_signaling(const NavigationState state) {
  switch (state) {
    case NavigationState::Free:
      digitalWrite(pin_led_free, HIGH);
      digitalWrite(pin_led_turning, LOW);
      digitalWrite(pin_led_critical, LOW);
      break;

    case NavigationState::Turn:
      digitalWrite(pin_led_free, LOW);
      digitalWrite(pin_led_turning, HIGH);
      digitalWrite(pin_led_critical, LOW);
      break;

    case NavigationState::Critical:
      digitalWrite(pin_led_free, LOW);
      digitalWrite(pin_led_turning, LOW);
      digitalWrite(pin_led_critical, HIGH);
      break;
  }
}

// Acionamento do buzzer piezoelétrico para emissao de alerta sonoro
void control_acoustic_alarm(const bool activate) {
  if (activate) {
    tone(pin_buzzer, buzzer_alarm_frequency_hz);
    return;
  }
  noTone(pin_buzzer);
}

// Retorna a descricao textual do estado de navegacao para a telemetria serial
const __FlashStringHelper* get_navigation_state_label(const NavigationState state) {
  switch (state) {
    case NavigationState::Free:
      return F("VIA LIVRE (AVANCANDO)");
    case NavigationState::Turn:
      return F("OBSTACULO DETECTADO (DESVIO / GIRO)");
    case NavigationState::Critical:
      return F("PROXIMIDADE CRITICA (MARCHA A RE)");
  }
  return F("INDEFINIDO");
}

// Retorna a descricao textual do acionamento dos motores para a telemetria serial
const __FlashStringHelper* get_motor_action_label(const NavigationState state) {
  switch (state) {
    case NavigationState::Free:
      return F("FRENTE / FRENTE");
    case NavigationState::Turn:
      return F("RE / FRENTE");
    case NavigationState::Critical:
      return F("RE / RE");
  }
  return F("PARADO");
}

// Transmissao periodica das informacoes do veiculo pela porta serial
void transmit_telemetry(const float distance_cm, const NavigationState state) {
  Serial.print(F("[TELEMETRIA] Distancia: "));
  Serial.print(distance_cm, telemetry_decimals);
  Serial.print(F(" cm | Estado: "));
  Serial.print(get_navigation_state_label(state));
  Serial.print(F(" | Motores: "));
  Serial.println(get_motor_action_label(state));
}
}  // namespace

void setup() {
  Serial.begin(serial_baud_rate);
  Serial.println(F("=================================================="));
  Serial.println(F(" CARRO ROBOTICO AUTONOMO - ARDUINO UNO            "));
  Serial.println(F(" Status: Inicializado com Sucesso                 "));
  Serial.println(F("=================================================="));

  pinMode(pin_ultrasonic_trig, OUTPUT);
  pinMode(pin_ultrasonic_echo, INPUT);

  pinMode(pin_motor_left_forward, OUTPUT);
  pinMode(pin_motor_left_backward, OUTPUT);
  pinMode(pin_motor_right_forward, OUTPUT);
  pinMode(pin_motor_right_backward, OUTPUT);

  pinMode(pin_led_free, OUTPUT);
  pinMode(pin_led_turning, OUTPUT);
  pinMode(pin_led_critical, OUTPUT);
  pinMode(pin_buzzer, OUTPUT);

  // Inicializacao dos atuadores em condicao segura
  stop_drive_motors();
  control_acoustic_alarm(false);

  // Amostragem inicial e partida deterministica
  current_distance_cm = read_ultrasonic_distance_cm();
  current_state = determine_navigation_state(current_distance_cm);

  control_drive_motors(current_state);
  update_visual_signaling(current_state);
  control_acoustic_alarm(current_state == NavigationState::Critical);
}

void loop() {
  const unsigned long current_ms = millis();

  // Amostragem periodica do sensor ultrassonico a cada 100 ms
  if (current_ms - last_sampling_ms >= sampling_interval_ms) {
    last_sampling_ms = current_ms;

    current_distance_cm = read_ultrasonic_distance_cm();
    current_state = determine_navigation_state(current_distance_cm);

    // Atualizacao imediata dos motores, LEDs e alarme sonoro
    control_drive_motors(current_state);
    update_visual_signaling(current_state);
    control_acoustic_alarm(current_state == NavigationState::Critical);
  }

  // Transmissao periodica da telemetria serial a cada 1 segundo (1000 ms)
  if (current_ms - last_telemetry_ms >= telemetry_interval_ms) {
    last_telemetry_ms = current_ms;
    transmit_telemetry(current_distance_cm, current_state);
  }
}
