/**
 * CAN Bus Node - Arduino Nano + MCP2515
 * Suporta modo Transmitter ou Receiver
 * Definir NODE_MODE no platformio.ini
 */

#include <SPI.h>
#include <Arduino.h>
#include <mcp2515.h>

// Configuração do nó (definir no platformio.ini)
// -DNODE_MODE=TRANSMITTER ou -DNODE_MODE=RECEIVER
#ifndef NODE_MODE
    #define NODE_MODE TRANSMITTER  // Padrão: Transmissor
#endif

#define TRANSMITTER 1
#define RECEIVER 2
#define BUZZER 3

MCP2515 mcp2515(10);
struct can_frame canMsg;

// ═══════════════════════════════════════════════════════
// CÓDIGO DO TRANSMISSOR
// ═══════════════════════════════════════════════════════
#if NODE_MODE == TRANSMITTER

unsigned long lastSendTime = 0;
uint32_t messageCount = 0;

void setup() {
    while (!Serial);
    Serial.begin(9600);
    
    Serial.println(F("=== TRANSMITTER MODE ==="));
    
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();
    
    // Teste de leitura do modo
    MCP2515::ERROR err = mcp2515.setNormalMode();
    if (err == MCP2515::ERROR_OK) {
        Serial.println("Modo Normal definido com SUCESSO.");
    } else {
        Serial.println("ERRO ao definir modo Normal! Verifique cabos SPI.");
    }
    
    Serial.println(F("MCP2515 inicializado!"));
}

void loop() {
    if (millis() - lastSendTime >= 1000) {
        lastSendTime = millis();
        messageCount++;
        
        struct can_frame frame;
        frame.can_id = 0x036;
        frame.can_dlc = 8;
        frame.data[0] = 0xDE;
        frame.data[1] = 0xAD;
        frame.data[2] = 0xBE;
        frame.data[3] = 0xEF;
        frame.data[4] = (messageCount >> 24) & 0xFF;
        frame.data[5] = (messageCount >> 16) & 0xFF;
        frame.data[6] = (messageCount >> 8) & 0xFF;
        frame.data[7] = messageCount & 0xFF;
        
        if (mcp2515.sendMessage(&frame) == MCP2515::ERROR_OK) {
            Serial.print(F("["));
            Serial.print(messageCount);
            Serial.print(F("] Enviado: 0x"));
            Serial.println(frame.can_id, HEX);
        }
    }
}

// ═══════════════════════════════════════════════════════
// CÓDIGO DO RECEPTOR
// ═══════════════════════════════════════════════════════
#elif NODE_MODE == RECEIVER

uint32_t messagesReceived = 0;

void setup() {
    Serial.begin(9600);
    
    Serial.println(F("=== RECEIVER MODE ==="));
    
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ);
    mcp2515.setNormalMode();
    
    // Teste de leitura do modo
    MCP2515::ERROR err = mcp2515.setNormalMode();
    if (err == MCP2515::ERROR_OK) {
        Serial.println("Modo Normal definido com SUCESSO.");
    } else {
        Serial.println("ERRO ao definir modo Normal! Verifique cabos SPI.");
    }

    Serial.println(F("MCP2515 inicializado! Aguardando..."));
}

void loop() {
    struct can_frame frame;
    
    // Tenta ler mensagem. A biblioteca verifica RX0 e RX1 automaticamente.
    if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
        messagesReceived++;
        
        Serial.print(F("["));
        Serial.print(messagesReceived);
        Serial.print(F("] Recebido ID: 0x"));
        Serial.print(frame.can_id, HEX);
        Serial.print(F(" DLC: "));
        Serial.print(frame.can_dlc);
        Serial.print(F(" | Dados: "));
        
        for (int i = 0; i < frame.can_dlc; i++) {
            if (frame.data[i] < 0x10) Serial.print(F("0")); // Zero padding
            Serial.print(frame.data[i], HEX);
            Serial.print(F(" "));
        }
        Serial.println();
    }
}

// ═══════════════════════════════════════════════════════
// CÓDIGO DO Buzzer Node
// ═══════════════════════════════════════════════════════
#elif NODE_MODE == BUZZER


#define BUZZER_PIN        3
#define BUZZ_INTERVAL_MS  10000UL
#define BUZZ_DURATION_MS  100UL

unsigned long lastBuzzTime  = 0;
unsigned long buzzStartTime = 0;
bool          buzzerActive  = false;

void setup() {
    Serial.begin(9600);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println(F("=== BUZZER NODE READY ==="));

    // Boot test buzz
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);
    lastBuzzTime = millis();   // ← reset timer AFTER boot buzz
}

void loop() {
    unsigned long now = millis();

    // --- Trigger every 30 s ---
    if (!buzzerActive && (now - lastBuzzTime >= BUZZ_INTERVAL_MS)) {
        buzzerActive  = true;
        buzzStartTime = now;
        lastBuzzTime  = now;
        digitalWrite(BUZZER_PIN, HIGH);
        Serial.println(F("BUZZ ON"));
    }

    // --- Turn off after 100 ms ---
    if (buzzerActive && (now - buzzStartTime >= BUZZ_DURATION_MS)) {
        buzzerActive = false;
        digitalWrite(BUZZER_PIN, LOW);
        Serial.println(F("BUZZ OFF"));
    }
}

#endif