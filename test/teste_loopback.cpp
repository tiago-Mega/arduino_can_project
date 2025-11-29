#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

// Configuração do Pino CS (Chip Select)
// No Arduino Nano geralmente é o pino 10. Confirme a sua ligação.
struct can_frame frame;
MCP2515 mcp2515(10); 

void setup() {
  Serial.begin(9600);
  while (!Serial); // Aguarda a porta série abrir
  
  Serial.println("--- INICIANDO TESTE DE LOOPBACK ---");

  // 1. Reiniciar o módulo
  mcp2515.reset();
  
  // 2. Configurar velocidade (CRUCIAL: Mantenha 16MHz pois confirmou os seus cristais)
  mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ);
  
  // 3. ATIVAR MODO LOOPBACK
  // Aqui é a magia: O chip desliga-se do cabo externo e liga a saída à entrada internamente
  mcp2515.setLoopbackMode();

  Serial.println("Modo Loopback ativado. A testar envio/rececao...");
}

void loop() {
  // Preparar uma mensagem de teste
  struct can_frame frameToSend;
  frameToSend.can_id = 0x123;
  frameToSend.can_dlc = 4;
  frameToSend.data[0] = 0xAA;
  frameToSend.data[1] = 0xBB;
  frameToSend.data[2] = 0xCC;
  frameToSend.data[3] = 0xDD;

  // --- PASSO 1: ENVIAR ---
  /* 
     No modo Loopback, o envio é bem-sucedido se o chip estiver vivo.
     Não precisa de outro Arduino para dar o ACK.
  */
  MCP2515::ERROR result = mcp2515.sendMessage(&frameToSend);
  
  Serial.print("Envio: ");
  if (result == MCP2515::ERROR_OK) {
    Serial.print("SUCESSO -> ");
  } else {
    Serial.print("FALHA (Erro: ");
    Serial.print(result);
    Serial.print(") -> ");
  }

  delay(100); // Pequena pausa para o chip processar

  // --- PASSO 2: LER ---
  /*
     Se o chip estiver bom, a mensagem que acabamos de enviar
     deve estar agora disponível para leitura no buffer.
  */
  if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
    // Verificar se os dados são os mesmos que enviamos (0xAA)
    if (frame.data[0] == 0xAA && frame.can_id == 0x123) {
        Serial.println("RECECAO CONFIRMADA! (Hardware OK)");
    } else {
        Serial.println("DADOS CORROMPIDOS!"); 
    }
  } else {
    Serial.println("NADA RECEBIDO (Verifique fios SPI: SCK, SI, SO, CS)");
  }

  Serial.println("--------------------------------------------------");
  delay(2000); // Repete a cada 2 segundos
}
