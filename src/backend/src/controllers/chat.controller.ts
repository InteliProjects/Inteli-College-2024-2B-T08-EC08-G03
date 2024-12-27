import { Controller, Post, Body } from '@nestjs/common';
import { GptService } from '../services/gpt.services';
import { ActionService } from '../services/action.service';

@Controller('chat')
export class ChatController {
  constructor(
    private readonly gptService: GptService,
    private readonly actionService: ActionService,
  ) {}

  @Post('message')
  async sendMessage(@Body('prompt') prompt: string): Promise<string> {
    const response = await this.gptService.generateText(prompt);

    if (response.action) {
      const actionResult = await this.actionService.handleAction(response.action);
      return `${response.response}\n\nAction Result: ${actionResult}`;
    }

    return response.response;
  }
}
