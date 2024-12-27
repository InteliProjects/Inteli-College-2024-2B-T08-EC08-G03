import { Controller, Post, Body, HttpException, HttpStatus, Res } from '@nestjs/common';
import { ApiTags, ApiOperation, ApiResponse } from '@nestjs/swagger';
import { GptService } from '@services/gpt.services';
import { ActionService } from '@services/action.service'; // Import the ActionService
import { Twilio } from 'twilio';

@ApiTags('Twilio Webhook')
@Controller('twilio')
export class TwilioController {
  private readonly twilioClient: Twilio;

  constructor(
    private readonly gptService: GptService,
    private readonly actionService: ActionService, // Inject ActionService
  ) {
    this.twilioClient = new Twilio(
      process.env.TWILIO_ACCOUNT_SID,
      process.env.TWILIO_AUTH_TOKEN,
    );
  }

  @Post('webhook')
  @ApiOperation({ summary: 'Handle incoming messages from Twilio' })
  @ApiResponse({ status: 200, description: 'Message processed successfully.' })
  async handleIncomingMessage(@Body() body: any, @Res() res): Promise<any> {
    const { Body: incomingMessage, From: sender, To: receiver } = body;

    try {
      console.log('Incoming message:', incomingMessage);

      // Process message with GPT
      const gptResponse = await this.gptService.generateText(incomingMessage);
      console.log('Olha a mensagem!')
      let replyMessage: string;
      console.log(gptResponse)

      const message = await this.twilioClient.messages.create({
        body: gptResponse.response,
        from: receiver, // Twilio phone number
        to: sender,     // User's phone number
      });

      if (gptResponse.action) {
        console.log('Realizará Ação')
        const actionResult = await this.actionService.handleAction(gptResponse.action);
        console.log(actionResult);
      } else {
        // If no action, just send the GPT response
        replyMessage = gptResponse.response;
        console.log(replyMessage)
        console.log('eapenas enviando mensagem')
      }

      console.log('Sent message:', message.sid);

      return res.status(200).send('<Response></Response>');
    } catch (error) {
      console.error('Error processing message:', error);
      throw new HttpException(
        'Failed to process message',
        HttpStatus.INTERNAL_SERVER_ERROR,
      );
    }
  }
}
