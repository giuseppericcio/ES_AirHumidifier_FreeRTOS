# Versione del progetto senza FreeRTOS #

## Getting Started ##
Per eseguirlo clonare la repository e dare i seguenti comandi:
 - make clean (per eliminare eventuali file di compilazioni precedenti)
 - make prepare (prepariamo i file necessari per la compilazione, tra cui le librerie dell'HAL per la STM32F3 presenti nel repository ufficiale fornito dalla ST al seguente [link](https://github.com/STMicroelectronics/STM32CubeF3))
 - make (crea i file sorgente del progetto linkando le opportune librerie richieste)
 - make debug (installare OpenOCD per effettuare il debug del programma tramite il tool GDB)