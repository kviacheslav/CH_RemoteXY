# CH_RemoteXY
Проект с библиотекой RemoteXY
+ повтор remotexy->init(), если модуль не включился в работу, например, когда роутер еще не готов после отключения электросети.
+ повтор  remotexy->initCloud() со сбросом роутера, если нет интернета, например, когда роутер завис или gsm modem потерял связь со станцией. 
