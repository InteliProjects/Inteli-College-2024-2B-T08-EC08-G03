import { Module } from '@nestjs/common';
import { ConfigModule } from '@nestjs/config';

// Controllers
import { AppController } from '@controllers/app.controller';
import { GptController } from '@controllers/gpt.controller';
import { UsersController } from '@controllers/user.controller';
import { TwilioController } from '@controllers/wpp.controller';
import { ChatController } from '@controllers/chat.controller'; // Novo controlador para Chat com GPT
import { LogsController } from '@controllers/logs.controller';

// Services
import { AppService } from '@services/app.service';
import { GptService } from '@services/gpt.services'; // Serviço GPT
import { ActionService } from '@services/action.service'; // Serviço para ações específicas
import { SupabaseService } from '@services/supabase.service';
import { LogsService } from '@services/logs.service';
import { UsersService } from '@services/user.service';
import { PositionController } from '@controllers/position.controller';
// import { WatsonService } from '@services/watson.service'; // Descomente se usar futuramente

@Module({
  imports: [
    ConfigModule.forRoot({
      isGlobal: true, // Torna as variáveis de ambiente acessíveis globalmente
    }),
  ],
  controllers: [
    AppController,
    UsersController,
    GptController,
    TwilioController,
    ChatController, // Controlador de Chat com GPT
    LogsController,
    PositionController
  ],
  providers: [
    AppService,
    GptService,
    ActionService, // Serviço para gerenciar ações baseadas em respostas GPT
    SupabaseService,
    LogsService,
    UsersService
    // WatsonService, // Descomente se usar Watson futuramente
  ],
})
export class AppModule {}
