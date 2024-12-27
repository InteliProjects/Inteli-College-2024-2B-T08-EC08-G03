import { ApiProperty } from '@nestjs/swagger';

export class CreateUser {
  @ApiProperty({
    description: 'Nome do usuário',
    type: String,
  })
  name: string;

  @ApiProperty({
    description: 'Password do usuário',
    type: String,
  })
  password: string;

  @ApiProperty({
    description: 'Email do usuário',
    type: String,
  })
  email: string;
}

export class UpdateUser {
  @ApiProperty({
    description: 'Atualizar nome do usuário',
    type: String,
  })
  name: string;

  @ApiProperty({
    description: 'Atualizar password do usuário',
    type: String,
  })
  password: string;

  @ApiProperty({
    description: 'Atualizar email do usuário',
    type: String,
  })
  email: string;
}
