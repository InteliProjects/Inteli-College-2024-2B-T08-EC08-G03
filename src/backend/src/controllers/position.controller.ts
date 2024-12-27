import { Controller, Get, Post, Body, Param, NotFoundException } from '@nestjs/common';
import { SupabaseService } from '@services/supabase.service';
import { ActionService } from '@services/action.service';
import { Position } from '../interfaces/position.interface';
import { ApiOperation, ApiResponse } from '@nestjs/swagger';

@Controller('position')
export class PositionController {
  constructor(
    private readonly supabaseService: SupabaseService,
    private readonly actionService: ActionService,
  ) {}

  @Post()
  @ApiOperation({ summary: 'Registrar uma posição na tabela position' })
  @ApiResponse({
    status: 201,
    description: 'Posição registrada com sucesso.',
  })
  async createPosition(@Body() positionDto: Position) {
    return this.supabaseService.createPosition(positionDto);
  }

  @Get(':name')
  @ApiOperation({ summary: 'Obter uma posição pelo nome na tabela position' })
  @ApiResponse({
    status: 200,
    description: 'Posição encontrada com sucesso.',
  })
  @ApiResponse({
    status: 404,
    description: 'Posição não encontrada.',
  })
  async getPositionByName(@Param('name') name: string): Promise<Position | null> {
    return this.supabaseService.getPositionByName(name);
  }

  @Post('move')
  @ApiOperation({ summary: 'Mover o robô para a posição correspondente ao nome' })
  @ApiResponse({
    status: 200,
    description: 'Robô movido para a posição com sucesso.',
  })
  @ApiResponse({
    status: 404,
    description: 'Posição não encontrada.',
  })
  async moveRobot(@Body('name') name: string) {
    const position = await this.supabaseService.getPositionByName(name);
    if (!position) {
      throw new NotFoundException(`Posição com o nome '${name}' não encontrada.`);
    }
    console.log('valores no endpoint', position.x, position.y, position.orientation);
    const actionResponse = await this.actionService.handleAction({
      name: 'moveRobot',
      parameters: {
        x: position.x,
        y: position.y,
        orientation: position.orientation,
      },
    });

    return {
      message: `Robô movido para a posição '${name}' com sucesso.`,
      position,
      actionResponse,
    };
  }
}
