// import { Injectable, HttpException, HttpStatus } from '@nestjs/common';
// import { ConfigService } from '@nestjs/config';
// import { AssistantV2 } from 'ibm-watson/assistant/v2';
// import { IamAuthenticator } from '@ibm/cloud-sdk-core';

// @Injectable()
// export class WatsonService {
//   private readonly assistant: AssistantV2;
//   private readonly assistantId: string;

//   constructor(private configService: ConfigService) {
//     this.assistantId = this.configService.get<string>('WATSON_ASSISTANT_ID');
//     const apiKey = this.configService.get<string>('WATSON_API_KEY');
//     const serviceUrl = this.configService.get<string>('WATSON_URL');

//     // Initialize Watson Assistant
//     this.assistant = new AssistantV2({
//       version: '2021-06-14', // Use the latest Watson version
//       authenticator: new IamAuthenticator({ apikey: apiKey }),
//       serviceUrl: serviceUrl,
//     });
//   }

//   async sendMessage(sessionId: string, message: string): Promise<string> {
//     try {
//       // Send message to Watson Assistant
//       const response = await this.assistant.message({
//         assistantId: this.assistantId,
//         sessionId: sessionId,
//         input: {
//           message_type: 'text',
//           text: message,
//         },
//       });

//       const watsonResponse = response.result.output.generic
//         ?.map((item) => item.text)
//         .join('\n');

//       return watsonResponse || 'No response from Watson.';
//     } catch (error) {
//       console.error('Watson API Error:', error);
//       throw new HttpException(
//         'Failed to fetch response from Watson Assistant',
//         HttpStatus.INTERNAL_SERVER_ERROR,
//       );
//     }
//   }

//   async createSession(): Promise<string> {
//     try {
//       const response = await this.assistant.createSession({
//         assistantId: this.assistantId,
//       });
//       return response.result.session_id;
//     } catch (error) {
//       console.error('Watson Session Error:', error);
//       throw new HttpException(
//         'Failed to create Watson Assistant session',
//         HttpStatus.INTERNAL_SERVER_ERROR,
//       );
//     }
//   }

//   async deleteSession(sessionId: string): Promise<void> {
//     try {
//       await this.assistant.deleteSession({
//         assistantId: this.assistantId,
//         sessionId: sessionId,
//       });
//     } catch (error) {
//       console.error('Watson Session Deletion Error:', error);
//       // No need to throw error as this might be a cleanup operation
//     }
//   }
// }
