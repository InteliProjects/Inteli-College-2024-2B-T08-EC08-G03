import { ApiProperty } from '@nestjs/swagger';

export class ComandoDto {
  @ApiProperty({ description: 'Comando para o TurtleBot', example: 'ir para o ponto A' })
  comando: string;
}
