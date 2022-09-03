# MODBUS protocol simulation application

## Описание проекта
Проект по моделированию и имитации устройств MODBUS на ПК. При работе с промышленными и embedded устройствами возникает необходимость имитировать (симулировать) работу как master, так и slave устройств на ПК с загрузкой соответствующей карты регистров. Многие существующие программные продукты подходят для целого ряда задач такого плана, но не обеспечивают достаточной гибкости при отладке. В проекте реализована идея исследовать аспекты протокола MODBUS при моделировании с использованием ООП, обобщенного программирования и хранения данных в контейнерах C++.

## Состав проекта и используемые модули
- ModbusRegisterMap содержит класс для работы с картой регистров протокола MODBUS. Загрузка карты регистров из JSON (используется rapidjson), хранение, доступ и модификация.
- ModbusProtocolHandler описывает классы обработчики пакетов MODBUS для master и slave устройств. Подключается карта регистров ModbusRegisterMap, настраиваются функции-обработчики принимаемых и передаваемых пакетов.
- IndustryDataStreamsAL предоставляет единый интерфейс для различных физических и виртуальных устройств передачи данных (COM порт, Ethernet и т.д.).

## Версия 1.0
- ModbusProtocolHandler - slave реализован. Master на стадии тестирования и доработки.
- IndustryDataStreamsAL - реализована работа с COM портами ПК Windows. Далее необходима реализация DataStreamEthernet для работы с MODBUS TCP.
- ModbusRegisterMap и ModbusProtocolHandler реализованы в основном с помощью STL C++. При последующей доработке необходимо вынести отдельно некоторые обработчики для обеспечения полной переносимости между разными платформами.
- ModbusRegisterMap и ModbusProtocolHandler также запускались и тестировались на модуле с микроконтроллером STM32F779.
