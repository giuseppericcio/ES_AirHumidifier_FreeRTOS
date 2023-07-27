# Embedded Systems  - AirHumidifier 💨 con FreeRTOS 

Il presente documento si propone di illustrare il progetto di un'applicazione per la scheda **STM32F303VC**, finalizzato alla realizzazione di un umidificatore automatico. L'obiettivo principale di questo progetto è quello di sviluppare un'applicazione basata sul kernel FreeRTOS per la gestione efficace dei task associati a sensori di temperatura e umidità, rilevamento del livello dell'acqua e controllo di un nebulizzatore per l'umidificazione dell'ambiente circostante. Sarà pertanto possibile gestire in modo efficiente i diversi task, garantendo un controllo del nebulizzatore alle variazioni dei livelli di umidità. Sarà inoltre sviluppata un'analisi qualitativa e comparativa tra la versione costruita con FreeRTOS e un’altra in Bare Metal. 

🔍 *Descrizione dei componenti del progetto:*

- Sensori di temperatura e umidità per il monitoraggio dell'ambiente
- Sensore di livello dell'acqua per rilevare la presenza di acqua nel serbatoio
- Nebulizzatore per umidificare l'ambiente circostante

🎯 *Obiettivi principali del progetto:*

- Sviluppo di un'applicazione basata su FreeRTOS
- Implementazione di task per la lettura dei sensori e abilitazione del nebulizer
- Controllo del nebulizzatore in base ai livelli di umidità e temperatura rilevati
- Analisi comparativa con un'implementazione Bare Metal

💡 *Vantaggi dell'utilizzo di FreeRTOS:*

- Gestione efficiente dei task grazie al sistema di scheduling
- Facilità di sviluppo e organizzazione del codice
- Migliore reattività e controllo del nebulizzatore alle variazioni di umidità

🔧 *Strumenti e tecnologie utilizzati:*

- Scheda STM32F303VC
- Sensori di temperatura e umidità
- Sensore di livello dell'acqua
- Nebulizzatore
- Kernel FreeRTOS
- Linguaggio di programmazione C/C++

## Panoramica del sistema
Il sistema è composto da un sensore di temperatura e umidità, un sensore di livello dell'acqua e un nebulizzatore. L'obiettivo del sistema è mantenere livelli di umidità
ottimali nell'ambiente circostante attivando il nebulizzatore in maniera automatica e portando dopo un certo tempo di transizione l’umidità nell’ambiente intorno alle
percentuali indicate nel seguente infographic della National Asthma Council Australia.<br>
<br>
![image](https://github.com/giuseppericcio/ES_AirHumidifier_FreeRTOS/assets/7995055/7392f536-231e-40b6-b7be-5d963039dd87)

## Architettura complessiva
Collegando opportunamente tutte le componenti si mostra l’architettura complessiva del sistema: 
<br><br>
![image](https://github.com/giuseppericcio/ES_AirHumidifier_FreeRTOS/assets/7995055/cec28bef-5954-4ce6-814d-a29546ee7245)
Si mostra inoltre una panoramica di funzionamento generale del sistema:
![image](https://github.com/giuseppericcio/ES_AirHumidifier_FreeRTOS/assets/7995055/cc87017b-3ce7-4438-a9f0-f9d07642b15d)

## Implementazione: Air Humidifier 🌬️💧
Codice di esempio che mostra il funzionamento dell'applicazione dell'umidificatore.

### Main 🏠
Nel main.c vengono inizializzate le periferiche necessarie all'applicazione tramite le opportune funzioni fornite dal produttore del microcontrollore (ST). Vengono poi creati, nell'ordine, il semaforo binario DHT_SEM e le 3 code per lo scambio dei messaggi di abilitazione del nebulizzatore. Infine, vengono creati i 4 task che implementano le funzioni di monitoraggio dei sensori e gestione del nebulizzatore stesso.

```c
int main(void)
{
  // ... Codice di inizializzazione delle periferiche ...

  DHT_SEM = xSemaphoreCreateBinary();
  xQueue1 = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint8_t));
  xQueue2 = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint8_t));
  xQueue3 = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint8_t));

  xTaskCreate(DHT11_Task, "DHT11", 128, NULL, 4, &DHT11_Handler);
  xTaskCreate(WaterLevel_Task, "WaterLevel", 128, NULL, 3, &WaterLevel_Handler);
  xTaskCreate(HandNeb_Task, "HandNeb", 128, NULL, 2, &HandNeb_Handler);
  xTaskCreate(Nebulizer_Task, "Nebulizer", 128, NULL, 1, &Nebulizer_Handler);

  // ... Altre inizializzazioni ...

  vTaskStartScheduler();

  while (1)
  {
  }
}
```

### DHT11 Task 🌡️
Il task del DHT11 si occupa della lettura dei valori di temperatura e umidità dal sensore. Sulla base di questi valori, il task invia il bit opportuno al task dell'Handler tramite la xQueue1. Questa funzione viene regolata tramite l'accesso al semaforo binario creato nel main, ed in particolare, il task eseguirà solo quando il TIM1 all'interno della propria callback rilascia il semaforo.

```c
void DHT11_Task(void *argument)
{
  while (1)
  {
    // ... Lettura valori da sensore DHT11 ...

    if (xSemaphoreTake(DHT_SEM, 2500) == pdTRUE)
    {
      // ... Logica di controllo e impostazione valori in base a temperatura e umidità ...

      xQueueSend(xQueue1, &DHT11_BUFFER[0], portMAX_DELAY);
    }
  }
}
```
### Water Level Task 💧
Il task si occupa di leggere il valore del livello d'acqua all'interno della vasca di pescaggio del nebulizzatore. Il sensore necessita di una conversione analogico-digitale per leggere tale livello. In base al valore letto, se l'acqua è presente o meno, il task invia l'opportuno bit al task dell'Handler tramite la xQueue2.

```c
void WaterLevel_Task(void *argument)
{
  while (1)
  {
    // ... Lettura valore del livello d'acqua tramite conversione ADC ...

    if (ADC_VAL >= 200UL)
    {
      WATERLEVEL_BUFFER[0] = 1UL;
    }
    else
    {
      WATERLEVEL_BUFFER[0] = 0UL;
    }

    xQueueSend(xQueue2, &WATERLEVEL_BUFFER[0], portMAX_DELAY);

    vTaskDelay(500);
  }
}
```

### Handler Nebulizer Task 🤖
L'Handler riceve i due bit dai task del DHT11 e del livello d'acqua. Se entrambi i bit sono alti, allora bisogna accendere il nebulizzatore, inviando sulla xQueue3 il bit di NEBULIZER_EN pari a 1. Altrimenti, il nebulizzatore viene spento, inviando il bit pari a 0 (l'Handler si comporta come una AND logica).

```c
void HandNeb_Task(void *argument)
{
  while (1)
  {
    xQueueReceive(xQueue1, &DHT11_EN, portMAX_DELAY);
    xQueueReceive(xQueue2, &WATERLEVEL_EN, portMAX_DELAY);

    if (DHT11_EN == 1UL && WATERLEVEL_EN == 1UL)
    {
      NEBULIZER_BUFFER[0] = 1UL;
    }
    else
    {
      NEBULIZER_BUFFER[0] = 0UL;
    }

    xQueueSend(xQueue3, &NEBULIZER_BUFFER[0], portMAX_DELAY);
    vTaskDelay(250);
  }
}
```

### Nebulizer Task 💨
Questo task è responsabile dell'accensione vera e propria del nebulizzatore. In base al valore ricevuto sulla xQueue3 dall'Handler, abiliterà o disabiliterà il nebulizzatore ponendo il pin PB6 (impostato come output pin) alto o basso.

```c
void Nebulizer_Task(void *argument)
{
  while (1)
  {
    xQueueReceive(xQueue3, &NEBULIZER_EN, portMAX_DELAY);

    if (NEBULIZER_EN == 1UL)
    {
      Nebulizer_Activated = 1UL;
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    }
    else
    {
      Nebulizer_Activated = 0UL;
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    }

    vTaskDelay(250);
  }
}
```

L'esecuzione sarà automatizzato dal ***Makefile** come da schema gerarchico dei file utili per l'esecuzione corretta dell'applicazione
![image](https://github.com/giuseppericcio/ES_AirHumidifier_FreeRTOS/assets/7995055/1a75ec03-db48-4311-843f-57ac792650c8)


**Schema ad alto livello** 
![image](https://github.com/giuseppericcio/ES_AirHumidifier_FreeRTOS/assets/7995055/c344b04e-de82-4299-8c4d-5c4b7c20ad0a)

Per vedere l'intero codice e lo scheduling completo dei task, puoi consultare il link al repository GitHub: Codice Completo

## Conclusioni

In conclusione, l'utilizzo di FreeRTOS offre numerosi vantaggi, come una gestione efficiente delle risorse, supporto al multithreading, meccanismi di sincronizzazione e comunicazione predefiniti e la scalabilità del sistema il tutto personalizzabile. Questi vantaggi semplificano lo sviluppo di sistemi embedded complessi, migliorando l'efficienza e l'efficacia riducendo la possibilità di errori nella gestione delle risorse e nella sincronizzazione delle attività.

### Progetto realizzato per soli scopi dimostrativi e didattici ✅
Copyright © 2023 - Progetto realizzato per l'esame di Embedded Systems presso l'Università di Napoli, Federico II. Realizzato esclusivamente a fini dimostrativi e didattici.
*Autori*: **Antonio Romano** - **Giuseppe Riccio**



### Bibliografia 📚

- [FreeRTOS Real Time Kernel (RTOS)](https://www.freertos.org/RTOS.html)
- [GCC online documentation - Option Summary](https://gcc.gnu.org/onlinedocs/gcc/Option-Summary.html)
- [GNU Toolchain | Arm Developer](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain)
- [Binutils - ld Documentation: Memory Usage](https://sourceware.org/binutils/docs-2.40/ld.html#index-memory-usage)
- [GCC online documentation - Developer Options](https://gcc.gnu.org/onlinedocs/gcc/Developer-Options.html)

Questa è la bibliografia di riferimento utilizzata nel progetto.
