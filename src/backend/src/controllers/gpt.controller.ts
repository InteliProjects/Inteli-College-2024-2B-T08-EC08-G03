// src/controllers/gpt.controller.ts
import { Controller, Post, Body, HttpException, HttpStatus } from '@nestjs/common';
import { ApiTags, ApiOperation, ApiResponse } from '@nestjs/swagger';
import { GptService } from '@services/gpt.services';
import { ComandoDto } from '../dto/prompt.dto';

@ApiTags('GPT')
@Controller('gpt')
export class GptController {
  constructor(private readonly gptService: GptService) {}

  @Post('generate')
  @ApiOperation({ summary: 'Generate a response from GPT' })
  @ApiResponse({ status: 200, description: 'GPT response generated successfully.' })
  @ApiResponse({ status: 500, description: 'Failed to fetch response from GPT.' })
  async generateText(@Body() comandoDto: ComandoDto): Promise<string> {
    try {
      return await this.gptService.generateText(comandoDto.comando);
    } catch (error) {
      throw new HttpException('Failed to generate response from GPT', HttpStatus.INTERNAL_SERVER_ERROR);
    }
  }
}
