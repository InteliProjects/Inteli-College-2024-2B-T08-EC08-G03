import { ApiProperty } from '@nestjs/swagger';

export class CreateLogDto {
  @ApiProperty({
    description: 'Nome do usuário que gerou o log',
    type: String,
  })
  user: string;

  @ApiProperty({
    description: 'Ação que o usuário executou',
    type: String,
  })
  action: string;

  @ApiProperty({
    description: 'Categoria em que o pedido do user se enquadra',
    type: String,
  })
  category: string;

  @ApiProperty({
    description: 'Local em que o user pediu que fosse levado o pedido',
    type: String,
  })
  local: string;

  @ApiProperty({
    description: 'Status da ação que o usuário executou',
    type: String,
  })
  status: string;

  @ApiProperty({
    description: 'ID do usuário (opcional)',
    type: Number,
    required: false,
  })
  user_id?: number;
}

export class UpdateStatusLog {
    @ApiProperty({
      description: "Atualizar o campo status do Log",
      type: String,  
    })
    status: string; 
  }