EXECUÇÃO DO MODELO DA REDE

1 - Primeiro certifique-se que os arquivos mod's foram processados. Se não houver uma pasta chamada "x86_64" neste diretório (o mesmo onde estão localizados os arquivos network.py e main.py), entre no diretório "extern_functions_and_data/mods" pelo terminal e execute:
    
    > nrnivmodl

    Será gerada uma pasta chamada "x86_64" com arquivos referentes aos mods. Copie esta pasta e cole no diretório acima onde os arquivos network.py e main.py estão localizados. 


2a - Para rodar o modelo:
    
    > nrniv -python main.py


2b - Além do passo anterior, em paralelo você pode simular o modelo usando múltiplos núcleos/processadores por meio de:
    
    > mpiexec -n 4 nrniv -python -mpi main.py

você pode substituir o 4 com o número de núcleos (do processador) que você queira utilizar.

---

Estrutura do diretório:
- new Network:
    - main.py         # arquivo por onde começará a execução da simulação da rede
    - network.py      # principal classe onde estão definidas os métodos e dados do modelo da rede
    - README.md       # este arquivo :-)

    - extern functions and data:    # funções e dados da classe Network implementadas em arquivos distintos
       
        - simulate.py          ### 
        - buildSimConfig.py    # métodos e dados responsáveis pelos parâmetros da simulação
        - data.py              ###

        - buildPopulationParameters.py    ###
        - buildCellRules.py               ###
        - buildSynMechParams.py           # métodos e dados responsáveis pelos parâmetros da rede
        - buildStimParams.py              ###
        - buildCellConnRules.py           ###

        - mods:    ### arquivos .mod utilizados nos métodos rsCellRules, fsiCellRules e buildSynMechParams
            - GP.mod
            - Izhi2003b.mod
            - STN.mod
            - Str.mod
            - thalamus.mod
            - x86_64:
                - ...

    - x86_64:    # arquivos mods já processados
        - ...

    - not_used:    # códigos testes e/ou para serem usados futuramente
        - ...
