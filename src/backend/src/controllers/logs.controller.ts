import { Controller, Get, Post, Body, Param, Put, Delete, Query } from '@nestjs/common';
import { LogsService } from '@services/logs.service';
import { Log } from '../interfaces/log.interface'; 
import { CreateLogDto, UpdateStatusLog } from 'src/dto/log';
import { ApiOperation, ApiResponse } from '@nestjs/swagger';

@Controller('logs')
export class LogsController {
  constructor(private readonly logsService: LogsService) {}

  @Post()
  @ApiOperation({ summary: 'Registrar um novo log' })
  @ApiResponse({
    status: 201,
    description: 'Log registrado com sucesso.',
  })
  async createLog(@Body() createLogDto: CreateLogDto) {
    return this.logsService.createLog(
      createLogDto.user,
      createLogDto.action,
      createLogDto.category,
      createLogDto.local,
      createLogDto.status,
      createLogDto.user_id
    );
}

  @Get('list')
  async listLogs(): Promise<Log[]> {
    return this.logsService.listLogs();
  }

  @Get(':id')
  async getLog(@Param('id') id: number): Promise<Log> {
    return this.logsService.getLog(id);
  }

  @Put('update-status/:id')
  @ApiOperation({ summary: 'Atualizar o status de um log' })
  @ApiResponse({ status: 200, description: 'Status do log atualizado com sucesso.' })
  @ApiResponse({ status: 404, description: 'Log não encontrado.' })
  async updateLogStatus(
    @Param('id') id: number,
    @Body() updateLogStatus: UpdateStatusLog  
  ) {
    return this.logsService.updateLogStatus(id, updateLogStatus.status);
  }
  

  @Delete(':id')
  async deleteLog(@Param('id') id: number) {
    return this.logsService.deleteLog(id);
  }
}
